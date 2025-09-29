#include <Arduino.h>
extern "C" {
  #include "driver/twai.h"
}

// ---- Pin mapping (match your wiring) ----
static const gpio_num_t TWAI_TX_GPIO = GPIO_NUM_7;  // ESP32-C3 -> MCP2551 TXD (Pin 1)
static const gpio_num_t TWAI_RX_GPIO = GPIO_NUM_6;  // MCP2551 RXD (Pin 4) -> divider -> ESP32-C3

// ---- Choose mode: if NO_ACK exists use it for 1-node isolation, else fallback to NORMAL ----
static twai_mode_t choose_mode_no_ack_fallback() {
  // Some Arduino-ESP32 versions don't expose TWAI_MODE_NO_ACK
  #ifdef TWAI_MODE_NO_ACK
    return TWAI_MODE_NO_ACK;      // best for "alone" transmit (no ACK needed)
  #else
    return TWAI_MODE_NORMAL;      // will BUS-OFF if no receiver; fine once second node is up
  #endif
}

static void printBusStatus(const char* tag) {
  twai_status_info_t s;
  if (twai_get_status_info(&s) == ESP_OK) {
    Serial.printf("[%s] state=%d tx_err=%u rx_err=%u tx_q=%u rx_q=%u\n",
                  tag, s.state, s.tx_error_counter, s.rx_error_counter, s.msgs_to_tx, s.msgs_to_rx);
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[Sender] Booting…");

  twai_mode_t mode = choose_mode_no_ack_fallback();
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_GPIO, TWAI_RX_GPIO, mode);
  twai_timing_config_t  t = TWAI_TIMING_CONFIG_500KBITS();         // 500 kbps (match both nodes)
  twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();       // accept all IDs

  esp_err_t err = twai_driver_install(&g, &t, &f);
  if (err != ESP_OK) {
    Serial.printf("[Sender] twai_driver_install failed: %d\n", err);
    while (true) delay(1000);
  }
  err = twai_start();
  if (err != ESP_OK) {
    Serial.printf("[Sender] twai_start failed: %d\n", err);
    while (true) delay(1000);
  }

  #ifdef TWAI_MODE_NO_ACK
    if (mode == TWAI_MODE_NO_ACK) Serial.println("[Sender] TWAI started in NO_ACK (solo test) @500 kbps.");
    else                          Serial.println("[Sender] TWAI started in NORMAL @500 kbps.");
  #else
    Serial.println("[Sender] TWAI started (NO_ACK not available in this SDK) in NORMAL @500 kbps.");
    Serial.println("          NOTE: If only this node is on the bus, it may go BUS_OFF (no ACK).");
  #endif

  printBusStatus("BUS");
}

void loop() {
  static uint32_t seq = 0;

  twai_message_t m = {};
  m.identifier = 0x123;          // Standard 11-bit ID
  m.extd = 0;                     // standard frame
  m.rtr = 0;                      // data frame
  m.data_length_code = 8;
  m.data[0] = (seq >> 0) & 0xFF;
  m.data[1] = (seq >> 8) & 0xFF;
  m.data[2] = 0xA5; m.data[3] = 0x5A; m.data[4] = 0x11; m.data[5] = 0x22; m.data[6] = 0x33; m.data[7] = 0x44;

  esp_err_t e = twai_transmit(&m, pdMS_TO_TICKS(100));
  Serial.printf("[TX] %s seq=%lu\n", (e==ESP_OK) ? "OK" : "FAIL", (unsigned long)seq);

  if (seq % 10 == 0) printBusStatus("BUS");

  seq++;
  delay(200);
}
