#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>  // for esp_wifi_set_promiscuous etc (not strictly needed)

static const char* AP_SSID   = "ESP_NOW_HUB";
static const char* AP_PASS   = "espnow123";   // >= 8 chars
static const uint8_t AP_CH   = 6;             // keep fixed for all nodes

// --- Packet we expect ---
#pragma pack(push,1)
struct Packet {
  uint8_t  node_id;     // 1..3
  uint16_t seq;         // ++ each send
  uint32_t t_ms;        // sender millis()
  float    value;       // test float
};
#pragma pack(pop)

static uint16_t last_seq[8]   = {0};
static uint32_t last_seen_ms[8] = {0};

static void onWiFiEvent(arduino_event_id_t event, arduino_event_info_t info) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_AP_START:
      Serial.println("[AP] Started");
      break;
    case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
      Serial.printf("[AP] STA joined: %02X:%02X:%02X:%02X:%02X:%02X\n",
        info.wifi_ap_staconnected.mac[0], info.wifi_ap_staconnected.mac[1], info.wifi_ap_staconnected.mac[2],
        info.wifi_ap_staconnected.mac[3], info.wifi_ap_staconnected.mac[4], info.wifi_ap_staconnected.mac[5]);
      break;
    case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
      Serial.printf("[AP] STA left:   %02X:%02X:%02X:%02X:%02X:%02X\n",
        info.wifi_ap_stadisconnected.mac[0], info.wifi_ap_stadisconnected.mac[1], info.wifi_ap_stadisconnected.mac[2],
        info.wifi_ap_stadisconnected.mac[3], info.wifi_ap_stadisconnected.mac[4], info.wifi_ap_stadisconnected.mac[5]);
      break;
    default:
      break;
  }
}

static void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len != (int)sizeof(Packet)) {
    Serial.printf("[NOW RX] from %02X:%02X:%02X:%02X:%02X:%02X len=%d (unexpected)\n",
                  mac[0],mac[1],mac[2],mac[3],mac[4],mac[5], len);
    return;
  }
  Packet p;
  memcpy(&p, data, sizeof(p));

  uint8_t idx = (p.node_id & 7);
  uint16_t prev = last_seq[idx];
  uint16_t drops = 0;
  if (prev != 0 || p.seq == 0) {
    uint16_t delta = (uint16_t)(p.seq - prev);
    if (delta > 1) drops = (uint16_t)(delta - 1);
  }
  last_seq[idx] = p.seq;
  last_seen_ms[idx] = millis();

  Serial.printf("node=%u seq=%u drops=%u t=%lu val=%.2f\n",
                p.node_id, p.seq, drops, (unsigned long)p.t_ms, p.value);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== Receiver: AP + ESP-NOW ===");

  WiFi.mode(WIFI_AP);
  WiFi.onEvent(onWiFiEvent);
  bool ok = WiFi.softAP(AP_SSID, AP_PASS, AP_CH, 0, 4);
  delay(200);
  Serial.printf("AP SSID: %s | PASS: %s | CH: %d\n", AP_SSID, AP_PASS, AP_CH);
  Serial.printf("AP MAC:  %s\n", WiFi.softAPmacAddress().c_str());
  Serial.printf("AP IP:   %s\n", WiFi.softAPIP().toString().c_str());

  if (esp_now_init() != ESP_OK) {
    Serial.println("esp_now_init FAILED");
    while (true) delay(1000);
  }
  esp_now_register_recv_cb(onDataRecv);
  Serial.println("Ready. Connect senders to this AP and start sending.");
}

void loop() {
  // Optional health ping
  static uint32_t t0 = 0;
  if (millis() - t0 > 500) {
    t0 = millis();
    for (uint8_t id = 1; id <= 3; ++id) {
      uint8_t idx = id & 7;
      if (last_seen_ms[idx] && (millis() - last_seen_ms[idx] > 200)) {
        Serial.printf("WARN: node=%u silent for %lums\n", id, (unsigned long)(millis() - last_seen_ms[idx]));
      }
    }
  }
}
