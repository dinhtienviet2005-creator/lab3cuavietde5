// Lab 3 - De 5: Fall Detection (ESP32 + MPU6050 + MQTT + FreeRTOS)
// Ho ten: tienviet    Lop: ........
// Task doc data (Core 1), Task canh bao (Core 1), Task truyen data (Core 0)
// Thu vien: Adafruit MPU6050, Adafruit Unified Sensor, Adafruit BusIO, PubSubClient

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <PubSubClient.h>

// ---------------- Phần cứng ----------------
#define LED_PIN     12
#define BUZZER_PIN  5
#define BUZZER_FREQ 2000

// ---------------- Mạng ----------------
const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PASS = "";
const char* MQTT_HOST = "broker.emqx.io";
const uint16_t MQTT_PORT = 1883;
// Topic riêng (phải trùng với index.html)
const char* TOPIC_DATA = "tienviet/falldetect/data";
const char* TOPIC_CMD  = "tienviet/falldetect/cmd";

// ---------------- Ngưỡng phát hiện ----------------
const float IMPACT_G   = 2.5;
const float FREEFALL_G = 0.4;
const float TILT_JUMP  = 45.0;
const float TILT_LYING = 60.0;
const uint32_t FALL_WINDOW_MS    = 2000;   // hai dấu hiệu cách nhau tối đa 2 s => té ngã
const uint32_t FALL_HOLD_MS      = 5000;   // giữ còi 5 s
const uint32_t WARN_HOLD_MS      = 3000;   // giữ đèn 3 s
const uint32_t MANUAL_TIMEOUT_MS = 10000;  // thủ công tự về tự động sau 10 s

// ---------------- Nhịp đọc ----------------
const uint32_t SAMPLE_PERIOD_MS = 50;
const uint8_t  REPORT_EVERY     = 10;      // 10 mẫu = 0,5 s
const int      TILT_HIST        = 20;      // 20 x 50 ms = 1 s
const float    G_MS2            = 9.80665;

// ---------------- Kiểu dữ liệu ----------------
typedef struct { float ax, ay, az, a, tilt, pitch, roll; } DuLieu;
typedef struct { float a, tilt; } CanhBao;
typedef enum { CMD_LED_ON, CMD_LED_OFF, CMD_BUZ_ON, CMD_BUZ_OFF, CMD_AUTO } Lenh;
typedef enum { R_NONE, R_IMPACT, R_TILT, R_FALL, R_MANUAL } LyDo;
typedef struct { bool led; bool buzzer; bool manual; uint8_t reason; } TrangThai;

QueueHandle_t qData, qAlert, qLenh, qTrangThai;
SemaphoreHandle_t semWiFi, mSerial;

Adafruit_MPU6050 mpu;
WiFiClient netClient;
PubSubClient mqtt(netClient);

// ---------------- Serial dùng chung (mutex) ----------------
void logLine(const char* fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  if (xSemaphoreTake(mSerial, pdMS_TO_TICKS(50)) == pdTRUE) {
    Serial.println(buf);
    xSemaphoreGive(mSerial);
  }
}

// ---------------- Sự kiện WiFi -> giơ cờ semWiFi ----------------
void onWiFiEvent(WiFiEvent_t event) {
  if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
    logLine("[WIFI] GOT_IP, IP = %s", WiFi.localIP().toString().c_str());
    xSemaphoreGive(semWiFi);
  } else if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    logLine("[WIFI] Mat ket noi WiFi");
  }
}

// ---------------- Nhận lệnh MQTT (chạy trong Task truyền) ----------------
void onMqttMessage(char* topic, byte* payload, unsigned int len) {
  char msg[24];
  unsigned int n = len < sizeof(msg) - 1 ? len : sizeof(msg) - 1;
  memcpy(msg, payload, n);
  msg[n] = '\0';

  Lenh cmd;
  if      (strcmp(msg, "led_on") == 0)     cmd = CMD_LED_ON;
  else if (strcmp(msg, "led_off") == 0)    cmd = CMD_LED_OFF;
  else if (strcmp(msg, "buzzer_on") == 0)  cmd = CMD_BUZ_ON;
  else if (strcmp(msg, "buzzer_off") == 0) cmd = CMD_BUZ_OFF;
  else if (strcmp(msg, "auto") == 0)       cmd = CMD_AUTO;
  else { logLine("[TRUYEN] Lenh khong hop le: %s", msg); return; }

  xQueueSend(qLenh, &cmd, 0);   // chỉ chuyển lệnh, không chạm LED/còi ở đây
  logLine("[TRUYEN] Nhan lenh: %s", msg);
}

// =====================================================================
// TASK ĐỌC DATA - Core 1, ưu tiên 2
// =====================================================================
void TaskDocData(void* pv) {
  TickType_t lastWake = xTaskGetTickCount();
  uint8_t dem = 0;
  float aDinh = 1.0f;   // gia tốc "bất thường nhất" trong 0,5 s gần đây
  for (;;) {
    sensors_event_t acc, gyr, nhiet;
    mpu.getEvent(&acc, &gyr, &nhiet);

    float ax = acc.acceleration.x / G_MS2;
    float ay = acc.acceleration.y / G_MS2;
    float az = acc.acceleration.z / G_MS2;
    float a  = sqrtf(ax * ax + ay * ay + az * az);

    // Góc nghiêng so với tư thế đứng thẳng (trục z hướng lên): 0° thẳng, 90° nằm ngang
    float cosTilt = (a > 0.05f) ? fmaxf(-1.0f, fminf(1.0f, az / a)) : 1.0f;
    float tilt  = acosf(cosTilt) * RAD_TO_DEG;
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
    float roll  = atan2f(ay, az) * RAD_TO_DEG;

    CanhBao c = { a, tilt };
    xQueueSend(qAlert, &c, 0);                          // mỗi mẫu

    if (fabsf(a - 1.0f) > fabsf(aDinh - 1.0f)) aDinh = a;

    if (++dem >= REPORT_EVERY) {                        // mỗi 0,5 s
      DuLieu d = { ax, ay, az, aDinh, tilt, pitch, roll };
      logLine("[DOC] a=%.2f g | ax=%.2f ay=%.2f az=%.2f | nghieng=%.1f | pitch=%.1f roll=%.1f",
              d.a, d.ax, d.ay, d.az, d.tilt, d.pitch, d.roll);
      if (xQueueSend(qData, &d, 0) != pdTRUE) {         // đầy (mất mạng): bỏ gói cũ, giữ gói mới
        DuLieu cu;
        xQueueReceive(qData, &cu, 0);
        xQueueSend(qData, &d, 0);
      }
      dem = 0;
      aDinh = 1.0f;
    }
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
  }
}

// =====================================================================
// TASK CẢNH BÁO - Core 1, ưu tiên 3 (chủ duy nhất của LED và còi)
// =====================================================================
static bool ledOn = false, buzOn = false;

void apDungDauRa(bool led, bool buz) {
  if (led && buz) led = false;                     // chỉ một trong hai, còi ưu tiên
  if (led != ledOn) { digitalWrite(LED_PIN, led ? HIGH : LOW); ledOn = led; }
  if (buz != buzOn) { if (buz) tone(BUZZER_PIN, BUZZER_FREQ); else noTone(BUZZER_PIN); buzOn = buz; }
}

void TaskCanhBao(void* pv) {
  float tiltHist[TILT_HIST];
  int nHist = 0, iHist = 0;
  bool coSoc = false, coNghieng = false;
  uint32_t socLuc = 0, nghiengLuc = 0;
  uint32_t fallUntil = 0, warnUntil = 0;
  uint8_t warnReason = R_NONE;
  bool manual = false, manLed = false, manBuz = false;
  uint32_t manualUntil = 0;
  float lastTilt = 0;

  TrangThai st = { false, false, false, R_NONE };
  xQueueOverwrite(qTrangThai, &st);

  for (;;) {
    CanhBao c;
    bool coDuLieu = xQueueReceive(qAlert, &c, pdMS_TO_TICKS(200)) == pdTRUE;
    uint32_t now = millis();

    if (coDuLieu) {
      lastTilt = c.tilt;

      // Dấu hiệu 1: gia tốc đột ngột
      if (c.a > IMPACT_G || c.a < FREEFALL_G) {
        coSoc = true; socLuc = now;
        warnUntil = now + WARN_HOLD_MS; warnReason = R_IMPACT;
      }
      // Dấu hiệu 2: góc nghiêng đổi đột ngột so với 1 giây trước
      if (nHist == TILT_HIST && fabsf(c.tilt - tiltHist[iHist]) > TILT_JUMP) {
        coNghieng = true; nghiengLuc = now;
        warnUntil = now + WARN_HOLD_MS; warnReason = R_TILT;
      }
      tiltHist[iHist] = c.tilt;
      iHist = (iHist + 1) % TILT_HIST;
      if (nHist < TILT_HIST) nHist++;

      if (coSoc && (now - socLuc) > FALL_WINDOW_MS) coSoc = false;
      if (coNghieng && (now - nghiengLuc) > FALL_WINDOW_MS) coNghieng = false;

      // TÉ NGÃ: đủ hai dấu hiệu trong 2 giây
      if (coSoc && coNghieng) {
        fallUntil = now + FALL_HOLD_MS;
        manual = false;                                  // té ngã giành lại quyền tự động
        coSoc = coNghieng = false;
        for (int i = 0; i < TILT_HIST; i++) tiltHist[i] = c.tilt;
        logLine("[CANH BAO] PHAT HIEN TE NGA! a=%.2f g, nghieng=%.1f do", c.a, c.tilt);
      }
    }

    // Lệnh từ web
    Lenh cmd;
    while (xQueueReceive(qLenh, &cmd, 0) == pdTRUE) {
      if (cmd == CMD_AUTO) { manual = false; continue; }
      if (!manual) { manLed = ledOn; manBuz = buzOn; }
      manual = true;
      manualUntil = now + MANUAL_TIMEOUT_MS;
      if (cmd == CMD_LED_ON)  { manLed = true;  manBuz = false; }
      if (cmd == CMD_LED_OFF) { manLed = false; }
      if (cmd == CMD_BUZ_ON)  { manBuz = true;  manLed = false; }
      if (cmd == CMD_BUZ_OFF) { manBuz = false; }
    }
    if (manual && (int32_t)(now - manualUntil) >= 0) manual = false;

    // Quyết định đầu ra
    bool led, buz;
    uint8_t reason;
    if (manual) {
      led = manLed; buz = manBuz; reason = R_MANUAL;
    } else {
      bool fall  = (int32_t)(fallUntil - now) > 0;
      bool warn  = (int32_t)(warnUntil - now) > 0;
      bool lying = lastTilt > TILT_LYING;
      if (fall)       { led = false; buz = true;  reason = R_FALL; }
      else if (warn)  { led = true;  buz = false; reason = warnReason; }
      else if (lying) { led = true;  buz = false; reason = R_TILT; }
      else            { led = false; buz = false; reason = R_NONE; }
    }
    apDungDauRa(led, buz);

    TrangThai moi = { ledOn, buzOn, manual, reason };
    if (moi.led != st.led || moi.buzzer != st.buzzer || moi.manual != st.manual || moi.reason != st.reason) {
      st = moi;
      xQueueOverwrite(qTrangThai, &st);
    }
  }
}

// =====================================================================
// TASK TRUYỀN DATA - Core 0, ưu tiên 1
// =====================================================================
const char* tenLyDo(uint8_t r) {
  switch (r) {
    case R_IMPACT: return "impact";
    case R_TILT:   return "tilt";
    case R_FALL:   return "fall";
    case R_MANUAL: return "manual";
    default:       return "none";
  }
}

bool ketNoiMQTT() {
  char clientId[32];
  snprintf(clientId, sizeof(clientId), "tienviet-fall-%04X", (unsigned)(esp_random() & 0xFFFF));
  if (mqtt.connect(clientId)) {
    mqtt.subscribe(TOPIC_CMD);
    logLine("[TRUYEN] Da ket noi MQTT broker %s", MQTT_HOST);
    return true;
  }
  logLine("[TRUYEN] Ket noi MQTT loi (rc=%d), thu lai sau 2 s", mqtt.state());
  return false;
}

void TaskTruyenData(void* pv) {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.onEvent(onWiFiEvent);
  WiFi.begin(WIFI_SSID, WIFI_PASS, 6);

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMqttMessage);
  mqtt.setBufferSize(512);
  mqtt.setSocketTimeout(5);
  uint8_t loiMqtt = 0;

  for (;;) {
    // Chưa có WiFi thì ngủ chờ cờ semWiFi
    if (WiFi.status() != WL_CONNECTED) {
      logLine("[TRUYEN] Cho WiFi... (status=%d)", WiFi.status());
      xSemaphoreTake(semWiFi, pdMS_TO_TICKS(2000));
      continue;
    }
    if (!mqtt.connected()) {
      if (!ketNoiMQTT()) {
        if (++loiMqtt >= 3) {                      // 3 lần liền thất bại: làm mới WiFi
          logLine("[TRUYEN] Lam moi ket noi WiFi");
          WiFi.disconnect();
          vTaskDelay(pdMS_TO_TICKS(500));
          WiFi.begin(WIFI_SSID, WIFI_PASS, 6);
          loiMqtt = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
        continue;
      }
      loiMqtt = 0;
    }

    mqtt.loop();                                   // giữ kết nối, nhận lệnh

    DuLieu d;
    if (xQueueReceive(qData, &d, pdMS_TO_TICKS(100)) == pdTRUE) {
      TrangThai st;
      xQueuePeek(qTrangThai, &st, 0);
      char json[256];
      snprintf(json, sizeof(json),
               "{\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,\"a\":%.2f,\"tilt\":%.1f,\"pitch\":%.1f,\"roll\":%.1f,"
               "\"led\":%d,\"buzzer\":%d,\"mode\":\"%s\",\"alert\":\"%s\"}",
               d.ax, d.ay, d.az, d.a, d.tilt, d.pitch, d.roll,
               st.led, st.buzzer, st.manual ? "manual" : "auto", tenLyDo(st.reason));
      if (mqtt.publish(TOPIC_DATA, json)) logLine("[TRUYEN] Gui: %s", json);
    }
  }
}

// =====================================================================
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(BUZZER_PIN, OUTPUT);

  qData      = xQueueCreate(5, sizeof(DuLieu));
  qAlert     = xQueueCreate(10, sizeof(CanhBao));
  qLenh      = xQueueCreate(3, sizeof(Lenh));
  qTrangThai = xQueueCreate(1, sizeof(TrangThai));
  semWiFi    = xSemaphoreCreateBinary();
  mSerial    = xSemaphoreCreateMutex();

  if (!mpu.begin()) {
    Serial.println("Khong tim thay MPU6050, kiem tra day SDA/SCL!");
    while (true) delay(1000);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("He thong khoi dong...");

  //                        hàm             tên           stack  tham số ưu tiên handle core
  xTaskCreatePinnedToCore(TaskCanhBao,    "CanhBao",    4096,  NULL,   3,     NULL,  1);
  xTaskCreatePinnedToCore(TaskDocData,    "DocData",    4096,  NULL,   2,     NULL,  1);
  xTaskCreatePinnedToCore(TaskTruyenData, "TruyenData", 8192,  NULL,   1,     NULL,  0);
}

void loop() {
  vTaskDelete(NULL);   // loopTask không dùng
}
