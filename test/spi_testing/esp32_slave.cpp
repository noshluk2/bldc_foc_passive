#include <Arduino.h>
#include <ESP32SPISlave.h>

static const int PIN_SCK  = 4;
static const int PIN_MISO = 5;  // Slave drives this back to Master
static const int PIN_MOSI = 6;  // Slave reads from Master here
static const int PIN_CS   = 7;  // Slave listens when CS is LOW

ESP32SPISlave slave;

uint8_t rxbuf[32];
uint8_t txbuf[32] = {'P','O','N','G'};

void setup() {
  Serial.begin(115200);
  delay(200);

  // Match the master’s mode (MODE1 here)
  slave.setDataMode(SPI_MODE1);
  // Bring up slave on the specified pins
  slave.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  // Queue first response so the master immediately receives something
  slave.queue(txbuf, 4);
  Serial.println("SPI Slave ready (MODE1). Waiting for master…");
}

void loop() {
  // When a master transaction completes, data becomes available
  if (slave.available()) {
    size_t n = slave.available();
    n = n > sizeof(rxbuf) ? sizeof(rxbuf) : n;
    slave.pop(rxbuf, n);

    Serial.print("Slave got: ");
    for (size_t i = 0; i < n; ++i) Serial.printf("%c ", isprint(rxbuf[i]) ? rxbuf[i] : '.');
    Serial.println();

    // Prepare next “PONG” reply
    slave.queue(txbuf, 4);
  }
}
