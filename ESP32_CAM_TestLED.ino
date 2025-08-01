#include <WiFi.h>
#include <PubSubClient.h>
#include "esp_camera.h"
#include <ArduinoJson.h>

// —————————————————————————————————————————————————
// Base64 Encoding Function Declaration
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
#define FLASH_GPIO_NUM     4 // Pin for the flash LED

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
const char* camera_id = "1"; // ⬅️⬅️⬅️ เปลี่ยน ID ของกล้องแค่ตรงนี้ที่เดียว
char topic_json_image[32];
char topic_status[32]; 

const char* msg_online   = "online";
const char* msg_offline  = "offline";

// —————————————————————————————————————————————————
// 6) Timing
// —————————————————————————————————————————————————
unsigned long lastCapture = 0;
const unsigned long captureInterval = 60000; // 1 minute

// —————————————————————————————————————————————————
// 7) Chunk size
// —————————————————————————————————————————————————
const size_t CHUNK_SIZE = 1024; // 1 KB per MQTT message

WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);

// Forward declarations
void setup_camera();
void connectWiFi();
void connectMQTT();
void publishImageAsJson(const uint8_t* data, size_t len);

void setup() {
  Serial.begin(115200);
  
  // Initialize flash pin
  pinMode(FLASH_GPIO_NUM, OUTPUT);
  digitalWrite(FLASH_GPIO_NUM, LOW); // Start with flash off
  
  delay(1000);
  Serial.println("\n🔌 Starting up...");

  // สร้าง Topic ต่างๆ จาก camera_id
  snprintf(topic_json_image, sizeof(topic_json_image), "camera/%s/image_json", camera_id);
  snprintf(topic_status, sizeof(topic_status), "camera/%s/status", camera_id);

  setup_camera();
  connectWiFi();

  mqttClient.setServer(mqtt_server, mqtt_port);
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

    // 1. เปิดแฟลช
    Serial.println("💡 Turning flash ON...");
    digitalWrite(FLASH_GPIO_NUM, HIGH);
    
    // เพิ่ม Delay ให้นานขึ้นเป็น 1 วินาทีเต็ม เพื่อให้ระบบไฟเสถียร
    Serial.println("Waiting 1000ms for power to stabilize...");
    delay(1000); 

    // 2. ✨ ล้าง Frame Buffer เก่าทิ้ง 2 ครั้งเพื่อความชัวร์
    Serial.println("Flushing buffer (1st time)...");
    camera_fb_t * flush_fb1 = esp_camera_fb_get();
    if (flush_fb1) {
        esp_camera_fb_return(flush_fb1);
    }
    delay(100); // หน่วงสั้นๆ

    Serial.println("Flushing buffer (2nd time)...");
    camera_fb_t * flush_fb2 = esp_camera_fb_get();
    if (flush_fb2) {
        esp_camera_fb_return(flush_fb2);
    }
    
    // 3. ถ่ายภาพจริง
    Serial.println("Capturing final frame...");
    camera_fb_t* fb = esp_camera_fb_get();
    
    if (!fb) {
      Serial.println("❌ Camera capture failed");
      digitalWrite(FLASH_GPIO_NUM, LOW); // ปิดแฟลชถ้าล้มเหลว
      esp_camera_deinit();
      delay(100);
      setup_camera();
      return;
    }

    // 4. ส่งภาพ
    publishImageAsJson(fb->buf, fb->len);
    esp_camera_fb_return(fb);

    // 5. ปิดแฟลช
    Serial.println("💡 Turning flash OFF...");
    digitalWrite(FLASH_GPIO_NUM, LOW);
  }
}

void publishImageAsJson(const uint8_t* data, size_t len) {
  size_t numChunks = (len + CHUNK_SIZE - 1) / CHUNK_SIZE;

  Serial.printf("📸 Preparing to send %u-byte image in %u JSON chunks...\n", (unsigned)len, (unsigned)numChunks);

  for (size_t i = 0; i < numChunks; i++) {
    size_t offset = i * CHUNK_SIZE;
    size_t chunkLen = min(CHUNK_SIZE, len - offset);

    String b64_chunk = base64_encode(data + offset, chunkLen);

    StaticJsonDocument<1536> doc;
    doc["id"] = camera_id;
    doc["index"] = i;
    doc["total"] = numChunks;
    doc["data"] = b64_chunk.c_str();

    char json_buffer[1536];
    size_t n = serializeJson(doc, json_buffer);
    
    if (mqttClient.publish(topic_json_image, json_buffer, n)) {
        Serial.printf("  📤 Sent chunk %d/%d (%d bytes JSON)\n", (int)i + 1, (int)numChunks, (int)n);
    } else {
        Serial.printf("  ❌ Failed to send chunk %d\n", (int)i + 1);
    }
    delay(50);
  }
  Serial.printf("✅ Finished sending image from camera %s.\n", camera_id);
}

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
  camera_config.xclk_freq_hz = 20000000;
  camera_config.pixel_format = PIXFORMAT_JPEG;
  camera_config.frame_size   = FRAMESIZE_HVGA; // 480×320
  camera_config.jpeg_quality = 6; // lower number = higher quality
  camera_config.fb_count     = 2; // Using 2 buffers is key to the problem/solution
  
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
    // กำหนด Last Will and Testament ก่อนทำการเชื่อมต่อ
    if (mqttClient.connect(camera_id, mqtt_user, mqtt_pass, topic_status, 1, true, msg_offline)) {
      
      Serial.println("✅ MQTT connected");

      // เมื่อเชื่อมต่อสำเร็จ ให้ประกาศสถานะ "online"
      mqttClient.publish(topic_status, msg_online, true); 

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
