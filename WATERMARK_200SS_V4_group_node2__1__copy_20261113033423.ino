#include <Arduino.h>
#include <math.h>
#include <WiFi.h>
#include <PubSubClient.h>

// ============================================================
// WATERMARK 200SS - V4 + MQTT
// แสดง Soil Matric Potential เป็นค่า kPa ติดลบ
// ส่งข้อมูลเข้า MQTT สำหรับ Dashboard เดิมและ Dashboard ใหม่
// ============================================================

// ===== 1. Wi-Fi และ MQTT =====
// ใส่รหัสผ่านของคุณเองก่อนอัปโหลด (ไม่ฝังรหัสผ่านจากไฟล์เดิม)
const char* WIFI_SSID     = "TP-Link_8872";
const char* WIFI_PASSWORD = "";

const char* MQTT_SERVER = " ";
const uint16_t MQTT_PORT  = 1883;
const char* MQTT_USER     = "";
const char* MQTT_PASSWORD = "";

// คง Topic เดิม เพื่อให้ Node-RED รับข้อมูลต่อได้
const char* MQTT_TOPIC        = "";
const char* MQTT_STATUS_TOPIC = "";

// เปลี่ยนเป็น SIMULATOR เมื่อใช้ A50Kx2 แทนเซนเซอร์จริง
const char* MEASUREMENT_MODE = "REAL_SENSOR";
const char* DEVICE_ID = "watermark_01";
const char* GROUP_ID = "node2";

WiFiClient espClient;
PubSubClient mqttClient(espClient);
uint32_t lastWiFiAttempt = 0;
uint32_t lastMQTTAttempt = 0;
uint32_t lastMeasurement = 0;
// วัดและส่งข้อมูล MQTT ทุก 60 วินาที ลดภาระของเซิร์ฟเวอร์
const uint32_t MEASUREMENT_INTERVAL_MS = 300000;

// ===== 2. ขา GPIO และวงจรเดิม =====
#define PIN1 26
#define PIN2 27
#define WM_ADC 36

// ตรวจสอบค่าตัวต้านทานจริงก่อนใช้งาน
const float R_FIXED   = 10000.0f;
const float VF_HIGH   = 3.33f;
const float VF_LOW    = 0.00f;
const float VR_HIGH   = 3.30f;
const float VR_LOW    = 0.041f;
const float CAL_SLOPE = 3.082f;
const float CAL_OFFSET = -0.083f;

// ===== 3. การวัดและคาลิเบรต =====
const uint32_t SAMPLE_DELAY_US = 20;
const int NUM_PAIRS = 10;
const float MAX_POLARITY_DIFF = 10.0f; // เตือนเท่านั้น ไม่ตัดค่า kPa
const float TOLERANCE = 0.01f;

const float REF_R[]   = {1.40f, 3.14f, 7.50f, 15.10f, 20.01f};
const float REF_KPA[] = {11.0f, 21.0f, 44.0f, 98.0f, 138.0f};
const int CAL_POINTS = sizeof(REF_R) / sizeof(REF_R[0]);

// ช่วงต่ำกว่า 11 kPa เป็นค่าประมาณ ยังต้องสอบเทียบจริง
const float R_SATURATED = 0.55f;
const float R_MIN_VALID = 0.30f;

uint32_t lastPulseTime = 0;

// ===== 4. หยุดจ่ายไฟให้เซนเซอร์ =====
void powerOff() {
  pinMode(PIN1, INPUT);
  pinMode(PIN2, INPUT);
}

// ===== 5. พัลส์สลับขั้วและอ่าน ADC =====
float readPulse(bool forward) {
  powerOff();
  digitalWrite(PIN1, LOW);
  digitalWrite(PIN2, LOW);
  pinMode(PIN1, OUTPUT);
  pinMode(PIN2, OUTPUT);

  if (forward) digitalWrite(PIN1, HIGH);
  else digitalWrite(PIN2, HIGH);

  uint32_t start = micros();
  delayMicroseconds(SAMPLE_DELAY_US);
  float voltage = analogReadMilliVolts(WM_ADC) / 1000.0f;
  powerOff();
  lastPulseTime = micros() - start;
  return voltage;
}

// ===== 6. ความต้านทาน (kOhm) -> ค่าแรงดึงน้ำ (kPa บวก) =====
float interpolateKpa(float r) {
  if (!isfinite(r) || r < R_MIN_VALID) return NAN;
  if (r <= R_SATURATED) return 0.0f;

  // ช่วง 0-11 kPa เป็นค่าประมาณ ไม่ใช่การคาลิเบรตจริง
  if (r < REF_R[0]) {
    return constrain((r - R_SATURATED) /
                     (REF_R[0] - R_SATURATED) * REF_KPA[0],
                     0.0f, 11.0f);
  }

  for (int i = 0; i < CAL_POINTS - 1; i++) {
    if (r >= REF_R[i] && r <= REF_R[i + 1]) {
      float ratio = (r - REF_R[i]) / (REF_R[i + 1] - REF_R[i]);
      return REF_KPA[i] + ratio * (REF_KPA[i + 1] - REF_KPA[i]);
    }
  }
  return NAN;
}

// ===== 7. เชื่อมต่อ Wi-Fi แบบไม่รีสตาร์ตบอร์ด =====
void maintainWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  if (lastWiFiAttempt != 0 && millis() - lastWiFiAttempt < 10000) return;
  lastWiFiAttempt = millis();
  Serial.println("WiFi: connecting...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

// ===== 8. เชื่อมต่อ MQTT และสถานะ ONLINE/OFFLINE =====
void maintainMQTT() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (mqttClient.connected()) {
    mqttClient.loop();
    return;
  }
  if (lastMQTTAttempt != 0 && millis() - lastMQTTAttempt < 5000) return;
  lastMQTTAttempt = millis();

  String clientId = String("esp32-watermark-") +
                    String((uint32_t)(ESP.getEfuseMac() & 0xFFFFFFFFULL), HEX);

  // หากการเชื่อมต่อหลุด Broker จะส่ง OFFLINE (Last Will)
  bool ok = mqttClient.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD,
                               MQTT_STATUS_TOPIC, 1, true, "OFFLINE");
  if (ok) {
    mqttClient.publish(MQTT_STATUS_TOPIC, "ONLINE", true);
    Serial.println("MQTT: connected");
  } else {
    Serial.print("MQTT: failed, state=");
    Serial.println(mqttClient.state());
  }
}

// ===== 9. ส่งข้อความ JSON ไป Topic เดิม =====
void publishPayload(const char* json) {
  Serial.println("========== MQTT DATA ==========");
  Serial.print("Topic: ");
  Serial.println(MQTT_TOPIC);
  Serial.print("Payload: ");
  Serial.println(json);

  if (WiFi.status() != WL_CONNECTED || !mqttClient.connected()) {
    Serial.println("MQTT publish: SKIPPED (disconnected)");
    return;
  }
  bool sent = mqttClient.publish(MQTT_TOPIC, json);
  Serial.println(sent ? "MQTT publish: SUCCESS" : "MQTT publish: FAILED");
}

// ===== 10. ส่ง ERROR โดยใช้ null แทนตัวเลข ไม่ใช่ศูนย์ =====
void publishInvalid(const char* reason) {
  char payload[350];
  snprintf(payload, sizeof(payload),
    "{\"device_id\":\"%s\",\"group_id\":\"%s\",\"mode\":\"%s\","
    "\"soil_kpa\":null,\"soil_matric_potential_kpa\":null,"
    "\"resistance_kohm\":null,\"soil_status\":\"UNKNOWN\","
    "\"online\":true,\"sensor_health\":\"ERROR\","
    "\"measurement_quality\":\"%s\",\"measurement_valid\":false}",
    DEVICE_ID, GROUP_ID, MEASUREMENT_MODE, reason);
  publishPayload(payload);
}

// ===== 11. ส่งผลวัดจริง รองรับ Dashboard เก่าและใหม่ =====
void publishMeasurement(float kpa, float resistanceK, const char* status,
                        float difference, float vF, float vR,
                        float pulseF, float pulseR) {
  bool mismatch = difference > MAX_POLARITY_DIFF;
  bool estimated = kpa < 11.0f;
  const char* health = (mismatch || estimated) ? "WARNING" : "OK";
  const char* quality = mismatch ? "POLARITY_MISMATCH" :
                        (estimated ? "LOW_RANGE_ESTIMATED" : "OK");

  // soil_kpa = ค่าบวกสำหรับ Dashboard เดิมเท่านั้น
  // soil_matric_potential_kpa = ค่าลบสำหรับ Dashboard ใหม่
  char payload[650];
  int len = snprintf(payload, sizeof(payload),
    "{\"device_id\":\"%s\",\"group_id\":\"%s\",\"mode\":\"%s\","
    "\"soil_kpa\":%.2f,\"soil_matric_potential_kpa\":%.2f,"
    "\"resistance_kohm\":%.3f,\"soil_status\":\"%s\","
    "\"online\":true,\"sensor_health\":\"%s\","
    "\"measurement_quality\":\"%s\",\"measurement_valid\":true,"
    "\"low_range_estimated\":%s,\"polarity_difference_pct\":%.1f,"
    "\"forward_voltage_v\":%.3f,\"reverse_voltage_v\":%.3f,"
    "\"forward_pulse_us\":%.1f,\"reverse_pulse_us\":%.1f}",
    DEVICE_ID, GROUP_ID, MEASUREMENT_MODE, kpa, -kpa, resistanceK, status,
    health, quality, estimated ? "true" : "false", difference,
    vF, vR, pulseF, pulseR);

  if (len < 0 || len >= (int)sizeof(payload)) {
    Serial.println("ERROR: MQTT payload too large");
    return;
  }
  publishPayload(payload);
}

// ===== 12. วัด 10 คู่พัลส์และคำนวณผล =====
void measureAndPublish() {
  float sumF = 0.0f, sumR = 0.0f;
  uint32_t timeF = 0, timeR = 0;

  for (int i = 0; i < NUM_PAIRS; i++) {
    sumF += readPulse(true);
    timeF += lastPulseTime;
    delay(5);
    sumR += readPulse(false);
    timeR += lastPulseTime;
    delay(5);
  }
  powerOff();

  float vF = CAL_SLOPE * (sumF / NUM_PAIRS) + CAL_OFFSET;
  float vR = CAL_SLOPE * (sumR / NUM_PAIRS) + CAL_OFFSET;
  float pulseF = (float)timeF / NUM_PAIRS;
  float pulseR = (float)timeR / NUM_PAIRS;

  Serial.println("\n--------------------------");
  Serial.printf("Forward A1: %.3f V\n", vF);
  Serial.printf("Reverse A1: %.3f V\n", vR);
  Serial.printf("Forward Pulse: %.1f us\n", pulseF);
  Serial.printf("Reverse Pulse: %.1f us\n", pulseR);

  if (!isfinite(vF) || !isfinite(vR) ||
      vF <= VF_LOW + 0.03f || vF >= VF_HIGH - 0.03f ||
      vR <= VR_LOW + 0.03f || vR >= VR_HIGH - 0.03f) {
    Serial.println("ERROR: Invalid voltage");
    Serial.println("POWER OFF");
    publishInvalid("INVALID_VOLTAGE");
    return;
  }

  float rForward = R_FIXED * (VF_HIGH - vF) / (vF - VF_LOW);
  float rReverse = R_FIXED * (vR - VR_LOW) / (VR_HIGH - vR);
  if (!isfinite(rForward) || !isfinite(rReverse) ||
      rForward <= 0.0f || rReverse <= 0.0f) {
    Serial.println("ERROR: Invalid resistance");
    Serial.println("POWER OFF");
    publishInvalid("INVALID_RESISTANCE");
    return;
  }

  float resistanceK = (rForward + rReverse) / 2000.0f;
  float difference = fabsf(rForward - rReverse) /
                     ((rForward + rReverse) / 2.0f) * 100.0f;

  Serial.printf("R Forward: %.3f kOhm\n", rForward / 1000.0f);
  Serial.printf("R Reverse: %.3f kOhm\n", rReverse / 1000.0f);
  Serial.printf("Resistance: %.3f kOhm\n", resistanceK);
  Serial.printf("Polarity Difference: %.1f %%\n", difference);
  if (difference > MAX_POLARITY_DIFF)
    Serial.println("WARNING: Polarity mismatch (ยังรายงาน kPa)");

  float rForCal = resistanceK;
  if (rForCal > REF_R[CAL_POINTS - 1] &&
      rForCal <= REF_R[CAL_POINTS - 1] * (1.0f + TOLERANCE)) {
    rForCal = REF_R[CAL_POINTS - 1];
    Serial.println("WARNING: Upper limit clamped");
  }

  float kpa = interpolateKpa(rForCal);
  if (!isfinite(kpa)) {
    Serial.println("OUTSIDE CALIBRATION RANGE");
    Serial.println("POWER OFF");
    publishInvalid("OUTSIDE_CALIBRATION_RANGE");
    return;
  }

  const char* status = (kpa < 30.0f) ? "WET" :
                       (kpa < 80.0f) ? "MODERATE" : "DRY";

  // Serial Monitor แสดงเฉพาะศักย์น้ำในดินเป็นค่าติดลบ
  Serial.printf("Soil Matric Potential: %.2f kPa\n", -kpa);
  Serial.print("Status: ");
  Serial.println(status);
  if (kpa < 11.0f)
    Serial.println("WARNING: Low-range estimated");
  Serial.println("POWER OFF");

  publishMeasurement(kpa, resistanceK, status, difference,
                     vF, vR, pulseF, pulseR);
}

// ===== 13. เริ่มต้นระบบ =====
void setup() {
  Serial.begin(115200);
  powerOff();
  pinMode(WM_ADC, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(WM_ADC, ADC_11db);
  analogRead(WM_ADC);

  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setBufferSize(768);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  lastWiFiAttempt = millis();

  Serial.println("WATERMARK 200SS - V4 + MQTT");
  Serial.println("SOIL MATRIC POTENTIAL (NEGATIVE kPa)");
  Serial.println("POWER OFF");
}

// ===== 14. ทำงานต่อเนื่อง =====
void loop() {
  maintainWiFi();
  maintainMQTT();

  if (millis() - lastMeasurement >= MEASUREMENT_INTERVAL_MS) {
    lastMeasurement = millis();
    measureAndPublish();
  }
  delay(2);
}
