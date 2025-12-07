#include "UdpSender.h"

bool UdpSender::begin(const char* ssid, const char* pass, const char* dest_ip, uint16_t dest_port, uint16_t local_port, uint32_t timeout_ms) {
  dest_port_ = dest_port;
  if (!dest_.fromString(dest_ip)) {
    return false;
  }

  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.begin(ssid, pass);

  if (!ensureConnection(timeout_ms)) {
    return false;
  }

  udp_.begin(local_port);  // 0 = auto
  return true;
}

bool UdpSender::ensureConnection(uint32_t timeout_ms) {
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < timeout_ms) {
    delay(200);
  }
  return WiFi.status() == WL_CONNECTED;
}

bool UdpSender::connected() const {
  return WiFi.status() == WL_CONNECTED;
}

bool UdpSender::send(const char* msg) {
  if (!msg || !connected()) return false;
  udp_.beginPacket(dest_, dest_port_);
  udp_.write(reinterpret_cast<const uint8_t*>(msg), strlen(msg));
  return udp_.endPacket() == 1;
}

bool UdpSender::send(const String& msg) {
  return send(msg.c_str());
}

bool UdpSender::send(float value, uint8_t decimals) {
  String s(value, decimals);
  return send(s);
}

bool UdpSender::send(double value, uint8_t decimals) {
  String s(value, decimals);
  return send(s);
}

bool UdpSender::send(int value) {
  String s(value);
  return send(s);
}

bool UdpSender::send(unsigned long value) {
  String s(value);
  return send(s);
}
