#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <SPI.h>
#include <SimpleFOC.h>
#include <math.h>   // fminf/fmaxf

// ---------- CONFIG ----------
#define NODE_ID            1     // change per device: 1,2,3,...
#define START_OFFSET_MS    0     // stagger: 0, 6, 13, 19 ms etc
#define SEND_TICKS         0     // 0 = send radians, 1 = send raw ticks (0..16383) as float

// ---------- AP (Receiver) credentials ----------
static const char* AP_SSID = "ESP_NOW_HUB";
static const char* AP_PASS = "espnow123";

// ---------- AS5048A SPI pins (ESP32-C3 SuperMini) ----------
static const int PIN_CS   = 7;  // CS
static const int PIN_SCK  = 4;  // SCK
static const int PIN_MISO = 5;  // MISO
static const int PIN_MOSI = 6;  // MOSI

// AS5048A via SimpleFOC
MagneticSensorSPI sensor(AS5048_SPI, PIN_CS);

// ---------- ESP-NOW packet ----------
#pragma pack(push,1)
struct Packet {
  uint8_t  node_id;
  uint16_t seq;
  uint32_t t_ms;
  float    value;     // radians (if SEND_TICKS=0) or ticks (if SEND_TICKS=1)
};
#pragma pack(pop)

// ---------- Radio state ----------
static uint8_t  peer_mac[6] = {0};
static bool     peer_ready  = false;
static uint16_t seq         = 0;

static const float K_RAD2TICKS = 16384.0f / (2.0f * PI);

// ---------- Helpers ----------
static bool addPeer(const uint8_t mac[6], uint8_t channel) {
  esp_now_peer_info_t info{};
  memcpy(info.peer_addr, mac, 6);
  info.channel = channel;   // must match AP channel
  info.encrypt = false;
  if (esp_now_is_peer_exist(mac)) esp_now_del_peer(mac);
  return (esp_now_add_peer(&info) == ESP_OK);
}

// median without std::swap (no extra headers needed)
static inline float median3f(float a, float b, float c) {
  float mn = fminf(a, fminf(b, c));
  float mx = fmaxf(a, fmaxf(b, c));
  return (a + b + c) - mn - mx;
}

static inline float wrap_0_2pi(float x) {
  // keep in [0, 2π)
  while (x >= 2.0f * PI) x -= 2.0f * PI;
  while (x <  0.0f)      x += 2.0f * PI;
  return x;
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
  sensor.init(&SPI);

  // ---- Wi-Fi join to lock channel ----
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
    Serial.println("Failed to join AP. Rebooting...");
    delay(1000);
    ESP.restart();
  }

  uint8_t ch = WiFi.channel();
  Serial.printf("Joined AP. CH=%u  IP=%s\n", ch, WiFi.localIP().toString().c_str());
  memcpy(peer_mac, WiFi.BSSID(), 6);
  Serial.printf("Peer (AP) MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                peer_mac[0],peer_mac[1],peer_mac[2],peer_mac[3],peer_mac[4],peer_mac[5]);

  if (esp_now_init() != ESP_OK) {
    Serial.println("esp_now_init FAILED. Rebooting...");
    delay(1000);
    ESP.restart();
  }
  if (!addPeer(peer_mac, ch)) {
    Serial.println("esp_now_add_peer FAILED. Rebooting...");
    delay(1000);
    ESP.restart();
  }
  peer_ready = true;

  // stagger to reduce collisions
  delay(START_OFFSET_MS);

  Serial.println("Setup complete. Streaming at 50 Hz…");
}

void loop() {
  if (!peer_ready) return;

  // Take 3 quick reads to reject single-frame glitches
  sensor.update(); float a1 = sensor.getAngle();   // radians
  sensor.update(); float a2 = sensor.getAngle();
  sensor.update(); float a3 = sensor.getAngle();

  float a_med = median3f(a1, a2, a3);
  static float a_ema = a_med;
  const float alpha = 0.30f;           // low-pass smoothing
  // handle wrap-around smoothly
  float err = a_med - a_ema;
  if (err >  PI)  err -= 2.0f * PI;
  if (err < -PI)  err += 2.0f * PI;
  a_ema = wrap_0_2pi(a_ema + alpha * err);

  float payload = SEND_TICKS
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
    p.value   = payload;

    esp_now_send(peer_mac, (const uint8_t*)&p, sizeof(p));
  }
}
