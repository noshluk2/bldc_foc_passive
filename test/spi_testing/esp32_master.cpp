#include <Arduino.h>
#include <SPI.h>

static const int PIN_SCK  = 4;
static const int PIN_MISO = 5;
static const int PIN_MOSI = 6;
static const int PIN_CS   = 7;

void setup() {
  Serial.begin(115200);
  delay(200);

  SPI.end(); // ensure clean state
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH); // idle HIGH

  Serial.println("SPI Master ready (MODE1).");
}

void loop() {
  static const uint8_t tx[4] = {'P','I','N','G'};
  uint8_t rx[4] = {0};

  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE1)); // 500 kHz, MODE1
  digitalWrite(PIN_CS, LOW);

  for (int i = 0; i < 4; ++i) {
    rx[i] = SPI.transfer(tx[i]); // full-duplex: send + receive per byte
  }

  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();

  Serial.printf("Master sent: %c %c %c %c | got: %c %c %c %c\n",
                tx[0],tx[1],tx[2],tx[3], rx[0],rx[1],rx[2],rx[3]);

  delay(500);
}
