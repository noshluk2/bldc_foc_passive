// Wi-Fi UDP sender: connects to your LAN AP and emits heartbeat packets once per second.
// Prints Wi-Fi status to serial, auto-reconnects, and targets DEST_IP:DEST_PORT.
// Handy for verifying PC listeners/firewall rules before ESP-NOW usage.
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// ---- Configure these before flashing ----
static const char* WIFI_SSID   = "Upper";
static const char* WIFI_PASS   = "786123786";
static const char* DEST_IP     = "192.168.1.4"; // laptop/PC IP on same LAN
static const uint16_t DEST_PORT = 4444;          // UDP port for the Python listener

WiFiUDP udp;

static void printStatus() {
  wl_status_t st = WiFi.status();
  if (st == WL_CONNECTED) {
    Serial.printf("[wifi] connected: IP=%s RSSI=%d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
  } else {
    Serial.printf("[wifi] status=%d (not connected)\n", (int)st);
  }
}

static bool connectWiFi() {
  Serial.printf("Connecting to '%s' ...\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false); // avoid flash writes
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    Serial.print('.');
    delay(300);
  }
  Serial.println();
  printStatus();
  return WiFi.status() == WL_CONNECTED;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("\n=== WiFi UDP sender test ===");
   Serial.printf("Target: %s:%u\n", DEST_IP, DEST_PORT);

  // Try a few times up front
  for (int i = 0; i < 3 && WiFi.status() != WL_CONNECTED; ++i) {
    if (connectWiFi()) break;
    Serial.println("Retrying WiFi...");
    delay(1000);
  }

  // Bind a source port (0 = auto)
  udp.begin(0);
}

void loop() {
  static uint32_t counter = 0;
  static uint32_t lastSend = 0;
  static uint32_t lastStatus = 0;
  uint32_t now = millis();

  // Periodic status on serial
  if (now - lastStatus >= 5000) {
    lastStatus = now;
    printStatus();
  }

  // Reconnect if dropped
  if (WiFi.status() != WL_CONNECTED) {
    if (!connectWiFi()) {
      delay(1000);
      return;
    }
    udp.stop();
    udp.begin(0);
    return;
  }

  // Send a UDP packet every 1s
  if (now - lastSend >= 1000) {
    lastSend = now;
    ++counter;

    char payload[64];
    snprintf(payload, sizeof(payload),
             "count=%lu millis=%lu rssi=%d",
             (unsigned long)counter,
             (unsigned long)now,
             WiFi.RSSI());

    udp.beginPacket(DEST_IP, DEST_PORT);
    udp.write((const uint8_t*)payload, strlen(payload));
    udp.endPacket();

    Serial.printf("[sent] %s\n", payload);
  }
}
