// Lightweight WiFi UDP sender helper.
// Call begin(...) once, then send(...) any text/number; handles reconnect attempts.
#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

class UdpSender {
 public:
  bool begin(const char* ssid, const char* pass, const char* dest_ip, uint16_t dest_port, uint16_t local_port = 0, uint32_t timeout_ms = 15000);
  bool connected() const;

  bool send(const char* msg);
  bool send(const String& msg);
  bool send(float value, uint8_t decimals = 3);
  bool send(double value, uint8_t decimals = 3);
  bool send(int value);
  bool send(unsigned long value);

 private:
  bool ensureConnection(uint32_t timeout_ms);

  WiFiUDP udp_;
  IPAddress dest_;
  uint16_t dest_port_ = 0;
};
