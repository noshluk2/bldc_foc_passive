#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#define NODE_ID           1   // 1, 2, or 3
#define START_OFFSET_MS   0   // 0, 6, or 13 (per node to stagger)

static const char* AP_SSID = "ESP_NOW_HUB";
static const char* AP_PASS = "espnow123";

#pragma pack(push,1)
struct Packet {
  uint8_t  node_id;
  uint16_t seq;
  uint32_t t_ms;
  float    value;
};
#pragma pack(pop)

static uint8_t peer_mac[6] = {0};
static bool    peer_ready = false;
static uint16_t seq = 0;

// simple 0.00..999.99 float for testing
static float randomFloat3Digits() {
  int v = random(0, 100000);  // 0..99999
  return v / 100.0f;
}

static bool addPeer(const uint8_t mac[6], uint8_t channel) {
  esp_now_peer_info_t info{};
  memcpy(info.peer_addr, mac, 6);
  info.channel = channel;   // must match AP channel
  info.encrypt = false;
  if (esp_now_is_peer_exist(mac)) esp_now_del_peer(mac);
  return (esp_now_add_peer(&info) == ESP_OK);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.printf("\n=== Sender | NODE=%d | OFFSET=%dms ===\n", NODE_ID, START_OFFSET_MS);

  // Seed random (unique-ish per device)
  uint32_t chip_id = 0;
  for (int i = 0; i < 17; ++i) chip_id = chip_id * 131 + (uint32_t)ESP.getCycleCount();
  randomSeed(chip_id ^ millis());

  // Join the AP so our radio locks to its channel
  WiFi.mode(WIFI_STA);
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
  Serial.printf("Joined. CH=%u  IP=%s\n", ch, WiFi.localIP().toString().c_str());

  // Receiver MAC is the AP BSSID we’re connected to
  memcpy(peer_mac, WiFi.BSSID(), 6);
  Serial.printf("Peer (AP) MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                peer_mac[0],peer_mac[1],peer_mac[2],peer_mac[3],peer_mac[4],peer_mac[5]);

  // Init ESP-NOW AFTER WiFi is up
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

  // Phase offset to reduce collisions
  delay(START_OFFSET_MS);
}

void loop() {
  if (!peer_ready) return;

  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last >= 20) {    // 50 Hz
    last = now;

    Packet p;
    p.node_id = (uint8_t)NODE_ID;
    p.seq     = ++seq;
    p.t_ms    = now;
    p.value   = randomFloat3Digits();

    esp_err_t r = esp_now_send(peer_mac, (const uint8_t*)&p, sizeof(p));
    // Optional: uncomment to debug send status
    // Serial.printf("send r=%d\n", (int)r);
  }
}
