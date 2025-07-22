#include <WiFi.h>
#include <PubSubClient.h>
#include "esp_camera.h"
#include <ArduinoJson.h> // <--- 1. เพิ่มไลบรารี ArduinoJson

// —————————————————————————————————————————————————
// Base64 Encoding Function
// ที่มา: https://github.com/Densaugeo/base64_arduino
// เราจะใส่ฟังก์ชันนี้เข้ามาในโค้ดเพื่อใช้เข้ารหัส
// —————————————————————————————————————————————————
String base64_encode(const uint8_t* data, size_t len);

// —————————————————————————————————————————————————
// 1) Pin definitions for AI‑Thinker ESP32‑CAM
// —————————————————————————————————————————————————
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// —————————————————————————————————————————————————
// 2) Your Wi‑Fi credentials
// —————————————————————————————————————————————————
const char* ssid     = "TP-Link_61E3_2.4";
const char* password = "0800453956";

// —————————————————————————————————————————————————
// 3) MQTT Broker settings
// —————————————————————————————————————————————————
const char* mqtt_server  = "192.168.1.104";
const uint16_t mqtt_port = 1883;
const char* mqtt_user    = "admin";
const char* mqtt_pass    = "admin1234";

// —————————————————————————————————————————————————
// 4) Camera config (global struct)
// —————————————————————————————————————————————————
camera_config_t camera_config;

// —————————————————————————————————————————————————
// 5) Identity & topics
// —————————————————————————————————————————————————
const char* camera_id = "1"; //เปลี่ยนตามเครื่องใช้งาน
// <--- 2. เราจะใช้ Topic เดียวสำหรับส่งข้อมูล JSON ทั้งหมด
char topic_json_image[32]; 

// —————————————————————————————————————————————————
// 6) Timing
// —————————————————————————————————————————————————
unsigned long lastCapture = 0;
const unsigned long captureInterval = 60000; // 1 minute

// —————————————————————————————————————————————————
// 7) Chunk size
// —————————————————————————————————————————————————
const size_t CHUNK_SIZE = 1024; // 1 KB per MQTT message

WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);

// Forward declarations
void setup_camera();
void connectWiFi();
void connectMQTT();
void publishImageAsJson(const uint8_t* data, size_t len);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n🔌 Starting up...");

  // <--- 3. เตรียม Topic สำหรับ JSON
  snprintf(topic_json_image, sizeof(topic_json_image), "camera/%s/image_json", camera_id);

  setup_camera();
  connectWiFi();

  mqttClient.setServer(mqtt_server, mqtt_port);
  // เพิ่มขนาด Buffer ของ MQTT Client เพื่อรองรับ JSON ที่อาจยาวขึ้น
  mqttClient.setBufferSize(CHUNK_SIZE + 512); 
  connectMQTT();
}

void loop() {
  if (!mqttClient.connected()) {
    connectMQTT();
  }
  mqttClient.loop();

  unsigned long now = millis();
  if (now - lastCapture >= captureInterval) {
    lastCapture = now;

    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("❌ Camera capture failed");
      esp_camera_deinit();
      delay(100);
      setup_camera();
      return;
    }

    // <--- 4. เรียกฟังก์ชันใหม่สำหรับส่งเป็น JSON
    publishImageAsJson(fb->buf, fb->len);
    esp_camera_fb_return(fb);
  }
}

// <--- 5. ฟังก์ชันใหม่สำหรับสร้างและส่ง JSON
void publishImageAsJson(const uint8_t* data, size_t len) {
  size_t numChunks = (len + CHUNK_SIZE - 1) / CHUNK_SIZE;

  Serial.printf("📸 Preparing to send %u-byte image in %u JSON chunks...\n", (unsigned)len, (unsigned)numChunks);

  for (size_t i = 0; i < numChunks; i++) {
    size_t offset = i * CHUNK_SIZE;
    size_t chunkLen = min(CHUNK_SIZE, len - offset);

    // เข้ารหัสชิ้นส่วนภาพเป็น Base64
    String b64_chunk = base64_encode(data + offset, chunkLen);

    // สร้าง JSON object
    StaticJsonDocument<1536> doc; // ขนาดต้องใหญ่พอสำหรับ Base64 (1024 * 1.33 ≈ 1365) + metadata
    doc["id"] = camera_id;
    doc["index"] = i;
    doc["total"] = numChunks;
    doc["data"] = b64_chunk.c_str();

    // แปลง JSON เป็น String เพื่อส่ง
    char json_buffer[1536];
    size_t n = serializeJson(doc, json_buffer);
    
    // ส่งข้อมูล JSON ผ่าน MQTT
    if (mqttClient.publish(topic_json_image, json_buffer, n)) {
        Serial.printf("  📤 Sent chunk %d/%d (%d bytes JSON)\n", (int)i + 1, (int)numChunks, (int)n);
    } else {
        Serial.printf("  ❌ Failed to send chunk %d\n", (int)i + 1);
    }

    delay(50); // หน่วงเวลาเล็กน้อยให้ Broker จัดการข้อความ
  }

  Serial.printf("✅ Finished sending image from camera %s.\n", camera_id);
}


// --- ส่วนของฟังก์ชันอื่นๆ (เหมือนเดิม) ---
void setup_camera() {
  Serial.println("🔌 Initializing camera...");
  camera_config.ledc_channel = LEDC_CHANNEL_0;
  camera_config.ledc_timer   = LEDC_TIMER_0;
  camera_config.pin_d0       = Y2_GPIO_NUM;
  camera_config.pin_d1       = Y3_GPIO_NUM;
  camera_config.pin_d2       = Y4_GPIO_NUM;
  camera_config.pin_d3       = Y5_GPIO_NUM;
  camera_config.pin_d4       = Y6_GPIO_NUM;
  camera_config.pin_d5       = Y7_GPIO_NUM;
  camera_config.pin_d6       = Y8_GPIO_NUM;
  camera_config.pin_d7       = Y9_GPIO_NUM;
  camera_config.pin_xclk     = XCLK_GPIO_NUM;
  camera_config.pin_pclk     = PCLK_GPIO_NUM;
  camera_config.pin_vsync    = VSYNC_GPIO_NUM;
  camera_config.pin_href     = HREF_GPIO_NUM;
  camera_config.pin_sccb_sda = SIOD_GPIO_NUM;
  camera_config.pin_sccb_scl = SIOC_GPIO_NUM;
  camera_config.pin_pwdn     = PWDN_GPIO_NUM;
  camera_config.pin_reset    = RESET_GPIO_NUM;
  camera_config.xclk_freq_hz = 22000000;
  camera_config.pixel_format = PIXFORMAT_JPEG;
  camera_config.frame_size   = FRAMESIZE_HVGA; // 480×320
  camera_config.jpeg_quality = 6;            // lower number = higher quality
  camera_config.fb_count     = 2;
  if (esp_camera_init(&camera_config) != ESP_OK) {
    Serial.println("❌ Camera init failed! Halting.");
    while (true) { delay(1000); }
  }
  Serial.println("✅ Camera initialized");
}

void connectWiFi() {
  Serial.printf("📶 Connecting to WiFi %s...", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.printf("\n✅ WiFi connected, IP: %s\n", WiFi.localIP().toString().c_str());
}

void connectMQTT() {
  Serial.printf("🔗 Connecting to MQTT %s:%u...", mqtt_server, mqtt_port);
  while (!mqttClient.connected()) {
    if (mqttClient.connect(camera_id, mqtt_user, mqtt_pass)) {
      Serial.println("✅ MQTT connected");
    } else {
      Serial.printf("❌ rc=%d, retry in 5s\n", mqttClient.state());
      delay(5000);
    }
  }
}

// —————————————————————————————————————————————————
// Base64 Encoding Implementation
// —————————————————————————————————————————————————
const char b64_alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                            "abcdefghijklmnopqrstuvwxyz"
                            "0123456789+/";

String base64_encode(const uint8_t* data, size_t len) {
  String encoded;
  encoded.reserve(((len + 2) / 3) * 4);

  for (size_t i = 0; i < len; i += 3) {
    uint32_t value = 0;
    value |= (i < len) ? (data[i] << 16) : 0;
    value |= (i + 1 < len) ? (data[i + 1] << 8) : 0;
    value |= (i + 2 < len) ? data[i + 2] : 0;

    encoded += b64_alphabet[(value >> 18) & 0x3F];
    encoded += b64_alphabet[(value >> 12) & 0x3F];

    if (i + 1 < len) {
      encoded += b64_alphabet[(value >> 6) & 0x3F];
    } else {
      encoded += '=';
    }

    if (i + 2 < len) {
      encoded += b64_alphabet[value & 0x3F];
    } else {
      encoded += '=';
    }
  }
  return encoded;
}
