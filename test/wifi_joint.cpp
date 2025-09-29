#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <SPI.h>
#include <SimpleFOC.h>

// ---------- CONFIG ----------
#define NODE_ID            3     // change per device: 1,2,3,4...
#define START_OFFSET_MS    13    // 0/6/13/19ms phase offsets
#define SEND_TICKS         1     // 1 = send raw SPI ticks (0..16383) as float; 0 = send radians

static const char* AP_SSID = "ESP_NOW_HUB";
static const char* AP_PASS = "espnow123";

// AS5048A SPI pins (ESP32-C3 SuperMini)
static const int PIN_CS   = 7;
static const int PIN_SCK  = 4;
static const int PIN_MISO = 5;
static const int PIN_MOSI = 6;

// Sensor: SimpleFOC AS5048 config (Mode 1, 14-bit)
MagneticSensorSPI sensor(AS5048_SPI, PIN_CS);

// ---------- Packet ----------
#pragma pack(push,1)
struct Packet {
  uint8_t  node_id;
  uint16_t seq;
  uint32_t t_ms;
  float    value;     // ticks (if SEND_TICKS=1) or radians
};
#pragma pack(pop)

// ---------- State ----------
static uint8_t  peer_mac[6] = {0};
static bool     peer_ready   = false;
static uint16_t seq          = 0;

const float K_RAD2TICKS = 16384.0f / (2.0f * PI);

// ---------- Helpers ----------
static bool addPeer(const uint8_t mac[6], uint8_t channel) {
  esp_now_peer_info_t info{};
  memcpy(info.peer_addr, mac, 6);
  info.channel = channel;   // must match AP channel
  info.encrypt = false;
  if (esp_now_is_peer_exist(mac)) esp_now_del_peer(mac);
  return (esp_now_add_peer(&info) == ESP_OK);
}

// Small utilities for filtering
static inline float clampAngle01(float a) {
  // keep angle in [0, 2π)
  while (a >= 2.0f * PI) a -= 2.0f * PI;
  while (a < 0.0f)       a += 2.0f * PI;
  return a;
}

static inline float median3(float a, float b, float c) {
  if (a > b) swap(a, b);
  if (b > c) swap(b, c);
  if (a > b) swap(a, b);
  return b;
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.printf("\n=== Sender AS5048A → ESP-NOW | NODE=%d | OFFSET=%dms | %s ===\n",
                NODE_ID, START_OFFSET_MS, SEND_TICKS ? "SENDING TICKS" : "SENDING RADIANS");

  // ---- SPI & Sensor ----
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);

  // If wiring is noisy, consider reducing SPI speed by editing the
  // SimpleFOC AS5048 config in library, or switch to shorter wires.
  sensor.init(&SPI);

  // ---- Join AP to lock channel ----
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);

  WiFi.begin(AP_SSID, AP_PASS);
  Serial.printf("Connecting to AP \"%s\" ...\n", AP_SSID);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(200);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Failed to join AP. Rebooting…");
    delay(1000);
    ESP.restart();
  }

  uint8_t ch = WiFi.channel();
  Serial.printf("Joined AP. CH=%u  IP=%s\n", ch, WiFi.localIP().toString().c_str());
  memcpy(peer_mac, WiFi.BSSID(), 6);
  Serial.printf("Peer (AP) MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                peer_mac[0],peer_mac[1],peer_mac[2],peer_mac[3],peer_mac[4],peer_mac[5]);

  if (esp_now_init() != ESP_OK) {
    Serial.println("esp_now_init FAILED. Rebooting…");
    delay(1000);
    ESP.restart();
  }
  if (!addPeer(peer_mac, ch)) {
    Serial.println("esp_now_add_peer FAILED. Rebooting…");
    delay(1000);
    ESP.restart();
  }
  peer_ready = true;

  delay(START_OFFSET_MS);
  Serial.println("Setup complete. Streaming at 50 Hz…");
}

void loop() {
  if (!peer_ready) return;

  // Take 3 rapid reads, median -> basic spike rejection
  sensor.update();
  float a1 = sensor.getAngle();
  sensor.update();
  float a2 = sensor.getAngle();
  sensor.update();
  float a3 = sensor.getAngle();

  float a_med = median3(a1, a2, a3);
  static float a_ema = a_med;               // exponential moving average
  const float alpha = 0.3f;                 // smoothing factor
  a_ema = clampAngle01(a_ema + alpha * (a_med - a_ema));

  float to_send = SEND_TICKS
                  ? (float)((uint16_t)((a_ema * K_RAD2TICKS + 0.5f)) & 0x3FFF)  // ticks as float
                  : a_ema;                                                     // radians

  // 50 Hz scheduler
  static uint32_t last_send = 0;
  uint32_t now = millis();
  if (now - last_send >= 20) {
    last_send = now;

    Packet p;
    p.node_id = (uint8_t)NODE_ID;
    p.seq     = ++seq;
    p.t_ms    = now;
    p.value   = to_send;

    esp_now_send(peer_mac, (const uint8_t*)&p, sizeof(p));

    // Local debug (comment if noisy)
    // if (SEND_TICKS) Serial.printf("ticks=%.0f\n", to_send); else Serial.printf("rad=%.6f\n", to_send);
  }
}
