/*
 * TX BRIDGE  (PC  --USB serial-->  this ESP32  --ESP-NOW-->  RX ESP32 --> arm)
 *
 * Transparent, line-based bridge. Every line the PC sends (ending in '\n')
 * is forwarded as ONE ESP-NOW packet. Anything the RX board sends back is
 * printed to the PC over serial.
 *
 * Your PC software (Python / Web Serial console) is unchanged: just select
 * THIS board's COM port at the same baud rate.
 *
 * Debug lines from this board start with '#', so the PC parser can ignore them.
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ================== USER CONFIG ==================
// MAC address of the RX (arm) ESP32  -> from get_mac.ino
uint8_t RX_MAC[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};

// Set true to send to ALL nearby receivers (e.g. dual-arm). No ACK/retry then.
#define USE_BROADCAST false

#define WIFI_CHANNEL 1          // must match RX
#define SERIAL_BAUD  115200     // must match what your PC software uses
#define LED_PIN      2          // onboard LED (blinks on successful delivery)
// =================================================

static uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static char    lineBuf[240];
static size_t  lineLen = 0;

static volatile uint32_t okCount = 0, failCount = 0;
static uint32_t lastReport = 0;

// ---- ESP32 Arduino core 2.x / 3.x compatible callbacks ----
#if defined(ESP_ARDUINO_VERSION) && ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 3, 0)
void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
#else
void onSent(const uint8_t *mac, esp_now_send_status_t status) {
#endif
  if (status == ESP_NOW_SEND_SUCCESS) {
    okCount++;
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  } else {
    failCount++;
  }
}

#if defined(ESP_ARDUINO_VERSION) && ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
#else
void onRecv(const uint8_t *mac, const uint8_t *data, int len) {
#endif
  // Feedback from the arm -> forward to PC untouched
  Serial.write(data, len);
}

void sendLine() {
  if (lineLen == 0) return;
  lineBuf[lineLen++] = '\n';
  const uint8_t *dest = USE_BROADCAST ? BROADCAST_MAC : RX_MAC;
  esp_err_t r = esp_now_send(dest, (uint8_t *)lineBuf, lineLen);
  if (r != ESP_OK) failCount++;
  lineLen = 0;
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(LED_PIN, OUTPUT);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("#ESP-NOW init FAILED");
    while (true) delay(1000);
  }
  esp_now_register_send_cb(onSent);
  esp_now_register_recv_cb(onRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, USE_BROADCAST ? BROADCAST_MAC : RX_MAC, 6);
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("#Failed to add peer");
    while (true) delay(1000);
  }

  Serial.print("#TX bridge ready. My MAC: ");
  Serial.println(WiFi.macAddress());
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      sendLine();
    } else if (lineLen < sizeof(lineBuf) - 2) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0;  // overflow -> drop garbage line
    }
  }

  // Link stats every 5 s (only useful with unicast; PC parser ignores '#')
  if (millis() - lastReport > 5000) {
    lastReport = millis();
    Serial.printf("#TX ok=%lu fail=%lu\n", (unsigned long)okCount, (unsigned long)failCount);
  }
}
