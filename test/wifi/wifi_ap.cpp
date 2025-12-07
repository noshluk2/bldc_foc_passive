// ESP-NOW receiver hub: starts a Wi-Fi AP and aggregates packets from joint nodes.
// Tracks latest seq/value per node_id and prints J1/J2/J3 lines at 50 Hz.
// Logs AP events (join/leave) and ignores unexpected payload sizes.
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

static const char* AP_SSID = "ESP_NOW_HUB";
static const char* AP_PASS = "espnow123";   // >= 8 chars
static const uint8_t AP_CH  = 6;            // keep fixed

// --- Packet from senders ---
#pragma pack(push,1)
struct Packet {
  uint8_t  node_id;   // 1..3
  uint16_t seq;       // ++ each send
  uint32_t t_ms;      // sender millis()
  float    value;     // encoder angle in radians [0, 2π)
};
#pragma pack(pop)

// Track 3 joints (indices 1..3 used)
static uint16_t last_seq[4]      = {0};
static uint32_t last_seen_ms[4]  = {0};
static float    last_angle[4]    = {NAN, NAN, NAN, NAN};

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
    // Unexpected payload size — ignore but log once
    Serial.printf("[NOW RX] from %02X:%02X:%02X:%02X:%02X:%02X len=%d (unexpected)\n",
                  mac[0],mac[1],mac[2],mac[3],mac[4],mac[5], len);
    return;
  }
  Packet p;
  memcpy(&p, data, sizeof(p));

  if (p.node_id < 1 || p.node_id > 3) return; // out of our range

  uint8_t idx = p.node_id; // 1..3
  // (Optional) drop tracking — not printed, but keeps stats internally
  uint16_t prev = last_seq[idx];
  if (prev == 0 || (uint16_t)(p.seq - prev) >= 1) {
    last_seq[idx] = p.seq;
    last_angle[idx] = p.value;          // store latest angle (radians)
    last_seen_ms[idx] = millis();       // update freshness
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== Receiver: AP + ESP-NOW | Consolidated J1,J2,J3 angles ===");

  WiFi.mode(WIFI_AP);
  WiFi.onEvent(onWiFiEvent);
  WiFi.softAP(AP_SSID, AP_PASS, AP_CH, 0, 4);
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

static inline void printAngleField(const char* label, float angle, bool fresh) {
  Serial.print(label);
  Serial.print('=');
  if (!fresh || isnan(angle)) {
    Serial.print("NA");
  } else {
    // print with 6 decimals; adjust if you want fewer
    Serial.print(angle, 6);
  }
}

void loop() {
  // Print a single consolidated line at 50 Hz (every 20 ms)
  static uint32_t last_print = 0;
  uint32_t now = millis();
  if (now - last_print >= 20) {
    last_print = now;

    // A value is "fresh" if seen within the last 200 ms
    bool f1 = (last_seen_ms[1] && (now - last_seen_ms[1] <= 200));
    bool f2 = (last_seen_ms[2] && (now - last_seen_ms[2] <= 200));
    bool f3 = (last_seen_ms[3] && (now - last_seen_ms[3] <= 200));

    printAngleField("J1", last_angle[1], f1);
    Serial.print(',');
    printAngleField("J2", last_angle[2], f2);
    Serial.print(',');
    printAngleField("J3", last_angle[3], f3);
    Serial.println();
  }
}
