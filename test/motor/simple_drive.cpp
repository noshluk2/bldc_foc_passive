#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <stdarg.h>

// --------- Select which test pattern to run ----------
#define TEST_MODE 2   // 1 = original slow pattern, 2 = 2s fwd/stop/rev/stop

// --------- ESP32 <-> MCF8316A pins (your mapping) ----------
#define PIN_DIR      6
#define PIN_SPEED    9   // PWM-capable
#define PIN_DRVOFF   10
#define PIN_BRAKE    21
#define PIN_EXT_CLK  4
#define PIN_EXT_WD   18
#define PIN_SOX      17   // analog output from driver (optional)
#define PIN_I2C_SDA  7
#define PIN_I2C_SCL  8

// --------- WiFi / UDP config ----------
static const char* WIFI_SSID    = "Upper";
static const char* WIFI_PASS    = "786123786";
static const char* DEST_IP      = "192.168.1.4";  // your laptop IP
static const uint16_t DEST_PORT = 4444;           // UDP port on laptop

WiFiUDP udp;

// --------- UDP logging helper ----------
void udpLog(const char* fmt, ...) {
  if (WiFi.status() != WL_CONNECTED) return;

  char buf[160];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  udp.beginPacket(DEST_IP, DEST_PORT);
  udp.write((const uint8_t*)buf, strlen(buf));
  udp.endPacket();
}

// --------- WiFi helper ----------
bool connectWiFi(uint32_t timeout_ms = 15000) {
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeout_ms) {
    delay(200);
  }
  if (WiFi.status() == WL_CONNECTED) {
    udp.begin(0);  // bind any local port
    IPAddress ip = WiFi.localIP();
    udpLog("BOOT: WiFi connected, IP=%u.%u.%u.%u\n", ip[0], ip[1], ip[2], ip[3]);
    return true;
  }
  return false;
}

// --------- PWM setup for SPEED pin ----------
static const int PWM_CHANNEL   = 0;
static const int PWM_FREQ_HZ   = 20000;  // 20 kHz
static const int PWM_RES_BITS  = 10;     // 0..1023
static const uint32_t PWM_MAX  = (1u << PWM_RES_BITS) - 1;

// speed levels
static const float SPEED_DUTY_FRACTION_SLOW   = 0.15f;  // 15% duty
static const float SPEED_DUTY_FRACTION_MEDIUM = 0.30f;  // 30% duty

static const uint32_t SPEED_DUTY_SLOW   = (uint32_t)(SPEED_DUTY_FRACTION_SLOW   * PWM_MAX);
static const uint32_t SPEED_DUTY_MEDIUM = (uint32_t)(SPEED_DUTY_FRACTION_MEDIUM * PWM_MAX);

// --------- simple state machine ----------
enum RunState {
  STATE_STOP1,
  STATE_FORWARD,
  STATE_STOP2,
  STATE_REVERSE
};

static RunState  state           = STATE_STOP1;
static uint32_t  state_start_ms  = 0;
static uint32_t  last_wifi_check = 0;

// helpers
static void setSpeedDuty(uint32_t duty) {
  if (duty > PWM_MAX) duty = PWM_MAX;
  ledcWrite(PWM_CHANNEL, duty);
}

static void setDirection(bool forward) {
  // polarity depends on wiring; adjust if reversed
  digitalWrite(PIN_DIR, forward ? LOW : HIGH);
}

// DRVOFF: HIGH = outputs off (coast), LOW = driver can drive motor
static void motorEnable(bool enable) {
  digitalWrite(PIN_DRVOFF, enable ? LOW : HIGH);
}

void setup() {
  // --- GPIO directions ---
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_SPEED, OUTPUT);
  pinMode(PIN_DRVOFF, OUTPUT);
  pinMode(PIN_BRAKE, OUTPUT);
  pinMode(PIN_EXT_CLK, OUTPUT);
  pinMode(PIN_EXT_WD, OUTPUT);
  pinMode(PIN_SOX, INPUT);  // driver analog out (we won't read it here)

  // --- Safe defaults ---
  motorEnable(false);             // keep OFF initially
  digitalWrite(PIN_BRAKE, LOW);   // no braking (depends on config)
  setDirection(true);             // default forward
  digitalWrite(PIN_EXT_CLK, LOW); // no external clock
  digitalWrite(PIN_EXT_WD, LOW);  // no watchdog tickle
  setSpeedDuty(0);                // 0% duty

  // --- WiFi connect ---
  connectWiFi(); // if it fails, we just won't see logs

  // --- PWM setup on SPEED pin ---
  ledcSetup(PWM_CHANNEL, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(PIN_SPEED, PWM_CHANNEL);
  setSpeedDuty(0);

  delay(500); // let driver power up

  // enable driver (still 0% duty)
  motorEnable(true);
  udpLog("Motor driver enabled, TEST_MODE=%d\n", TEST_MODE);

  state = STATE_STOP1;
  state_start_ms = millis();
}

void loop() {
  uint32_t now = millis();
  uint32_t elapsed = now - state_start_ms;

  // Check WiFi every 5 seconds; if dropped, try reconnect
  if (now - last_wifi_check >= 5000) {
    last_wifi_check = now;
    if (WiFi.status() != WL_CONNECTED) {
      if (connectWiFi()) {
        udpLog("Reconnected to WiFi\n");
      }
    }
  }

  // ----- TEST_MODE 1: original slow pattern -----
#if TEST_MODE == 1
  switch (state) {
    case STATE_STOP1:
      // Motor stopped for 3 seconds, then go forward (slow)
      setSpeedDuty(0);
      setDirection(true);
      if (elapsed > 3000) {
        udpLog("[MODE1] FORWARD (slow)\n");
        state = STATE_FORWARD;
        state_start_ms = now;
        setSpeedDuty(SPEED_DUTY_SLOW);
      }
      break;

    case STATE_FORWARD:
      // Spin forward for 4 seconds, then stop
      if (elapsed > 4000) {
        udpLog("[MODE1] STOP (after forward)\n");
        state = STATE_STOP2;
        state_start_ms = now;
        setSpeedDuty(0);
      }
      break;

    case STATE_STOP2:
      // Stop 3 seconds, then reverse (slow)
      if (elapsed > 3000) {
        udpLog("[MODE1] REVERSE (slow)\n");
        state = STATE_REVERSE;
        state_start_ms = now;
        setDirection(false);
        setSpeedDuty(SPEED_DUTY_SLOW);
      }
      break;

    case STATE_REVERSE:
      // Spin reverse 4 seconds, then back to STOP1
      if (elapsed > 4000) {
        udpLog("[MODE1] STOP (after reverse), restarting cycle\n");
        state = STATE_STOP1;
        state_start_ms = now;
        setSpeedDuty(0);
        setDirection(true);  // reset to forward for next cycle
      }
      break;
  }

  // ----- TEST_MODE 2: 2s forward, stop, reverse, stop (medium speed) -----
#elif TEST_MODE == 2
  switch (state) {
    case STATE_STOP1:
      // Motor stopped for 2 seconds, then go forward (medium)
      setSpeedDuty(0);
      setDirection(true);
      if (elapsed > 2000) {
        udpLog("[MODE2] FORWARD (2s, medium speed)\n");
        state = STATE_FORWARD;
        state_start_ms = now;
        setSpeedDuty(SPEED_DUTY_MEDIUM);
      }
      break;

    case STATE_FORWARD:
      // Spin forward for 2 seconds, then stop
      if (elapsed > 2000) {
        udpLog("[MODE2] STOP (after forward)\n");
        state = STATE_STOP2;
        state_start_ms = now;
        setSpeedDuty(0);
      }
      break;

    case STATE_STOP2:
      // Stop 2 seconds, then reverse (medium)
      if (elapsed > 2000) {
        udpLog("[MODE2] REVERSE (2s, medium speed)\n");
        state = STATE_REVERSE;
        state_start_ms = now;
        setDirection(false);
        setSpeedDuty(SPEED_DUTY_MEDIUM);
      }
      break;

    case STATE_REVERSE:
      // Spin reverse 2 seconds, then back to STOP1
      if (elapsed > 2000) {
        udpLog("[MODE2] STOP (after reverse), restarting cycle\n");
        state = STATE_STOP1;
        state_start_ms = now;
        setSpeedDuty(0);
        setDirection(true);  // reset to forward for next cycle
      }
      break;
  }
#endif
}
