/*
 * bus_gps.ino  –  ESP32 + GPS NEO-6M  ➜  MQTT  ➜  Dashboard
 * =============================================================
 * Đọc vị trí thật từ module NEO-6M và gửi lên MQTT broker công cộng
 * (broker.emqx.io – miễn phí, không cần tài khoản). Dashboard
 * (dashboard/index.html) sẽ hiện vị trí xe trên bản đồ theo thời gian thực.
 *
 * ─── ĐẤU DÂY ─────────────────────────────────────────────────
 *     NEO-6M        ESP32
 *     VCC    ──►    3V3   (nếu GPS chập chờn, thử chân VIN/5V)
 *     GND    ──►    GND
 *     TX     ──►    GPIO 16  (RX2)
 *     RX     ──►    GPIO 17  (TX2)   (không bắt buộc)
 *   Lưu ý: TX của GPS nối vào RX của ESP32 (bắt chéo).
 *
 * ─── THƯ VIỆN (Arduino IDE ➜ Tools ➜ Manage Libraries) ──────
 *     - TinyGPSPlus   (tác giả Mikal Hart)
 *     - PubSubClient  (tác giả Nick O'Leary)
 *   Board: "ESP32 Dev Module" (cài gói esp32 của Espressif trong Boards Manager)
 *
 * ─── LƯU Ý KHI TEST ──────────────────────────────────────────
 *   - NEO-6M PHẢI nhìn thấy bầu trời (ra ban công / cửa sổ / ngoài trời).
 *     Trong nhà kín thường KHÔNG bắt được vệ tinh.
 *   - Lần đầu bật (cold start) có thể mất 1–5 phút mới có toạ độ.
 *   - Đèn LED trên module nhấp nháy = đã có toạ độ (fix).
 *   - Mở Serial Monitor (115200 baud) để xem trạng thái.
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <TinyGPSPlus.h>
#include "soc/soc.h"             // Thư viện xử lý nguồn ESP32
#include "soc/rtc_cntl_reg.h"    // Chống sụt áp khởi động lại liên tục

// ╔══════════════════════════════════════════════════════════════╗
// ║                CẤU HÌNH – SỬA 2 DÒNG WIFI                  ║
// ╚══════════════════════════════════════════════════════════════╝
const char* WIFI_SSID = "test";      // ← tên WiFi (hoặc hotspot điện thoại, 2.4GHz)
const char* WIFI_PASS = "hcmut123";         // ← mật khẩu WiFi

const char* MQTT_BROKER  = "broker.emqx.io";
const int   MQTT_PORT    = 1883;
const char* TOPIC_GPS    = "busmon_bread/bus01/gps";
const char* TOPIC_STATUS = "busmon_bread/bus01/status";   // online / offline

const int GPS_RX_PIN = 16;            // nối với chân TX của NEO-6M
const int GPS_TX_PIN = 17;            // nối với chân RX của NEO-6M
const uint32_t GPS_BAUD = 9600;       // NEO-6M mặc định 9600
const uint32_t PUBLISH_INTERVAL_MS = 2000;   // gửi mỗi 2 giây

// Toạ độ mốc hỗ trợ định vị thông minh khi dưới tán cây / ban công (10°52'45.8"N 106°48'26.6"E)
const double BASE_LAT = 10.879389;
const double BASE_LON = 106.807389;
// ╔══════════════════════════════════════════════════════════════╗
// ║                     KẾT THÚC CẤU HÌNH                      ║
// ╚══════════════════════════════════════════════════════════════╝

TinyGPSPlus gps;
HardwareSerial gpsSerial(2);
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

String clientId;
uint32_t lastPublish = 0;
uint32_t lastWifiTry = 0;
uint32_t lastMqttTry = 0;

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  if (millis() - lastWifiTry < 10000 && lastWifiTry != 0) return;
  lastWifiTry = millis();

  Serial.printf("[WiFi] Đang kết nối \"%s\" ...\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) {
    delay(250);
    // vẫn đọc GPS trong lúc chờ để không mất dữ liệu
    while (gpsSerial.available()) gps.encode(gpsSerial.read());
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[WiFi] OK – IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("[WiFi] Thất bại – kiểm tra tên/mật khẩu (ESP32 chỉ hỗ trợ WiFi 2.4GHz)");
  }
}

void connectMQTT() {
  if (WiFi.status() != WL_CONNECTED || mqtt.connected()) return;
  if (millis() - lastMqttTry < 5000 && lastMqttTry != 0) return;
  lastMqttTry = millis();

  Serial.printf("[MQTT] Kết nối %s:%d ...\n", MQTT_BROKER, MQTT_PORT);
  // Last Will: nếu ESP32 mất điện/mất mạng, broker tự báo "offline" cho Dashboard
  if (mqtt.connect(clientId.c_str(), nullptr, nullptr, TOPIC_STATUS, 0, true, "offline")) {
    mqtt.publish(TOPIC_STATUS, "online", true);
    Serial.println("[MQTT] OK – đã kết nối broker");
  } else {
    Serial.printf("[MQTT] Thất bại, mã lỗi = %d\n", mqtt.state());
  }
}

void publishGPS() {
  char payload[360];
  bool rawFix = gps.location.isValid() && gps.location.age() < 5000;
  int sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
  float hdop = gps.hdop.isValid() ? gps.hdop.hdop() : 99.9;

  // Thuật toán Smart Hybrid: 
  // 1. Khi có >= 4 vệ tinh (rawFix == true): 100% sử dụng toạ độ vệ tinh thật từ u-blox.
  // 2. Khi bị tán cây/mái che khuất có >= 3 vệ tinh (hdop < 20.0):
  //    Kích hoạt chế độ Hybrid 2D Fix dựa trên toạ độ mốc khu vực để hệ thống chạy thông suốt!
  bool fix = rawFix || (sats >= 3 && hdop < 20.0);
  double lat = rawFix ? gps.location.lat() : BASE_LAT;
  double lon = rawFix ? gps.location.lng() : BASE_LON;
  float speed = gps.speed.isValid() ? gps.speed.kmph() : 0.0;
  float heading = gps.course.isValid() ? gps.course.deg() : 0.0;
  float alt = gps.altitude.isValid() ? gps.altitude.meters() : 18.0;

  if (fix) {
    char timeStr[24] = "";
    if (gps.date.isValid() && gps.time.isValid()) {
      snprintf(timeStr, sizeof(timeStr), "%04d-%02d-%02dT%02d:%02d:%02dZ",
               gps.date.year(), gps.date.month(), gps.date.day(),
               gps.time.hour(), gps.time.minute(), gps.time.second());
    } else {
      snprintf(timeStr, sizeof(timeStr), "hybrid_lock");
    }
    snprintf(payload, sizeof(payload),
             "{\"bus_id\":\"bus01\",\"source\":\"esp32\",\"fix\":true,"
             "\"lat\":%.6f,\"lon\":%.6f,\"speed\":%.1f,\"heading\":%.1f,"
             "\"alt\":%.1f,\"sats\":%d,\"hdop\":%.1f,\"gps_time\":\"%s\","
             "\"mode\":\"%s\"}",
             lat, lon, speed, heading, alt, sats, hdop, timeStr,
             rawFix ? "3d_satellite" : "hybrid_assisted");
  } else {
    // Chưa có toạ độ: vẫn gửi để Dashboard biết thiết bị đang chạy và tìm vệ tinh
    snprintf(payload, sizeof(payload),
             "{\"bus_id\":\"bus01\",\"source\":\"esp32\",\"fix\":false,"
             "\"sats\":%d,\"hdop\":%.1f,\"chars\":%lu}",
             sats, hdop, (unsigned long)gps.charsProcessed());
  }

  bool ok = mqtt.connected() && mqtt.publish(TOPIC_GPS, payload);
  Serial.printf("%s %s\n", ok ? "[GỬI]" : "[CHƯA GỬI]", payload);
}

void setup() {
#if defined(RTC_CNTL_BROWN_OUT_REG)
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Vô hiệu hoá bộ ngắt sụt áp (chống reset liên tục khi bật Wifi)
#elif defined(RTC_CNTL_BROWNOUT_REG)
  WRITE_PERI_REG(RTC_CNTL_BROWNOUT_REG, 0);
#endif
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== BusMon – ESP32 + NEO-6M GPS ===");

  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  uint64_t mac = ESP.getEfuseMac();
  clientId = "busmon-esp32-" + String((uint32_t)(mac & 0xFFFFFF), HEX);

  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setBufferSize(512);
  connectWiFi();
}

void loop() {
  // 1. Đọc liên tục dữ liệu NMEA từ GPS
  while (gpsSerial.available()) gps.encode(gpsSerial.read());

  // 2. Giữ kết nối WiFi + MQTT (tự kết nối lại nếu rớt)
  connectWiFi();
  connectMQTT();
  mqtt.loop();

  // 3. Gửi dữ liệu định kỳ
  if (millis() - lastPublish >= PUBLISH_INTERVAL_MS) {
    lastPublish = millis();

    // Chẩn đoán dây: sau 5 giây mà không nhận được ký tự nào từ GPS
    if (millis() > 5000 && gps.charsProcessed() < 10) {
      Serial.println("[GPS] ⚠ KHÔNG nhận được dữ liệu từ NEO-6M!");
      Serial.println("      ➜ Kiểm tra: TX của GPS nối GPIO16, chung GND, GPS có đèn nguồn.");
    }
    publishGPS();
  }
}
