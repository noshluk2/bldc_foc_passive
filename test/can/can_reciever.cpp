#include <Arduino.h>
extern "C" {
  #include "driver/twai.h"
}

static const gpio_num_t TWAI_TX_GPIO = GPIO_NUM_7;
static const gpio_num_t TWAI_RX_GPIO = GPIO_NUM_6;

static void printBusStatus(const char* tag) {
  twai_status_info_t s;
  if (twai_get_status_info(&s) == ESP_OK) {
    Serial.printf("[%s] state=%d tx_err=%u rx_err=%u tx_q=%u rx_q=%u\n",
                  tag, s.state, s.tx_error_counter, s.msgs_to_tx, s.msgs_to_rx);
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[Receiver] Booting…");

  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_GPIO, TWAI_RX_GPIO, TWAI_MODE_NORMAL);
  twai_timing_config_t  t = TWAI_TIMING_CONFIG_500KBITS();        // must match sender
  twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  ESP_ERROR_CHECK(twai_driver_install(&g, &t, &f));
  ESP_ERROR_CHECK(twai_start());

  Serial.println("[Receiver] Waiting for frames @500 kbps…");
  printBusStatus("BUS");
}

void loop() {
  twai_message_t rx;
  esp_err_t e = twai_receive(&rx, pdMS_TO_TICKS(500));  // wait up to 500 ms
  if (e == ESP_OK) {
    Serial.printf("[RX] id=0x%03X %s %s dlc=%d data:",
                  rx.identifier,
                  rx.extd ? "EXT" : "STD",
                  rx.rtr  ? "RTR" : "DATA",
                  rx.data_length_code);
    for (int i=0; i<rx.data_length_code; ++i) Serial.printf(" %02X", rx.data[i]);
    Serial.println();
  } else if (e != ESP_ERR_TIMEOUT) {
    Serial.printf("[RX] error=%d\n", e);
    printBusStatus("BUS");
  }

  static uint32_t last = 0;
  if (millis() - last > 2000) { printBusStatus("BUS"); last = millis(); }
}
