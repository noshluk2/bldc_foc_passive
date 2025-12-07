// AS5048A + Wi-Fi UDP sender
// - Reads encoder via SPI (SimpleFOC MagneticSensorSPI)
// - Sends ticks + angle_rad over UDP every 50 ms
// - Prints the same data to Serial in CSV-style

#include <Arduino.h>
#include <SimpleFOC.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// ---------- AS5048A SPI pins ----------
static const int PIN_CS   = 10;  // CS   , white
static const int PIN_SCK  = 12;  // SCK  , blue
static const int PIN_MISO = 13;  // MISO , green
static const int PIN_MOSI = 11;  // MOSI , yellow

MagneticSensorSPI sensor(AS5048_SPI, PIN_CS);

const uint32_t REPORT_MS     = 50;                        // 20 Hz
const float    K_RAD2TICKS   = 16384.0f / (2.0f * PI);    // rad -> ticks

// ---------- Wi-Fi / UDP config ----------
static const char* WIFI_SSID    = "Upper";
static const char* WIFI_PASS    = "786123786";
static const char* DEST_IP      = "192.168.1.4";  // laptop/PC IP on same LAN
static const uint16_t DEST_PORT = 4444;           // UDP port of Python listener

WiFiUDP udp;

// ---------- Wi-Fi helpers ----------
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
  WiFi.persistent(false);  // avoid flash writes
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

// ---------- Timing state ----------
static uint32_t last_report  = 0;
static uint32_t last_status  = 0;
static uint32_t sample_count = 0;

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println();
  Serial.println("=== AS5048A + WiFi UDP encoder sender ===");
  Serial.printf("Target: %s:%u\n", DEST_IP, DEST_PORT);

  // ----- SPI + encoder init -----
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  sensor.init(&SPI);

  Serial.println("ticks,angle_rad,millis,rssi");

  // ----- Wi-Fi connect -----
  for (int i = 0; i < 3 && WiFi.status() != WL_CONNECTED; ++i) {
    if (connectWiFi()) break;
    Serial.println("Retrying WiFi...");
    delay(1000);
  }

  // Bind a source port (0 = auto)
  udp.begin(0);
}

void loop() {
  uint32_t now = millis();

  // Periodic Wi-Fi status
  if (now - last_status >= 5000) {
    last_status = now;
    printStatus();
  }

  // Reconnect if Wi-Fi dropped
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[wifi] lost connection, reconnecting...");
    if (!connectWiFi()) {
      delay(1000);
      return;
    }
    udp.stop();
    udp.begin(0);
    return;
  }

  // ----- Encoder update -----
  sensor.update();
  float angle_rad = sensor.getAngle();                // [0, 2π)
  uint16_t ticks  = (uint16_t)(angle_rad * K_RAD2TICKS + 0.5f) & 0x3FFF;

  // ----- Report over Serial + UDP at 20 Hz -----
  if (now - last_report >= REPORT_MS) {
    last_report = now;
    ++sample_count;

    int rssi = WiFi.RSSI();

    // CSV line for Serial
    Serial.print(ticks);
    Serial.print(',');
    Serial.print(angle_rad, 9);
    Serial.print(',');
    Serial.print(now);
    Serial.print(',');
    Serial.println(rssi);

    // Build UDP payload (text)
    char payload[96];
    snprintf(payload, sizeof(payload),
             "ticks=%u,angle_rad=%.9f,millis=%lu,rssi=%d,sample=%lu",
             (unsigned)ticks,
             angle_rad,
             (unsigned long)now,
             rssi,
             (unsigned long)sample_count);

    udp.beginPacket(DEST_IP, DEST_PORT);
    udp.write((const uint8_t*)payload, strlen(payload));
    udp.endPacket();
  }
}
