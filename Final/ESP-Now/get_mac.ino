// Upload to EACH ESP32 (TX and RX) and open Serial Monitor @115200.
// Note down the MAC address of the RX board -> goes into tx_bridge.ino

#include <WiFi.h>
#include <esp_wifi.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(200);
  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());
}

void loop() {
  delay(2000);
  Serial.println(WiFi.macAddress());
}
