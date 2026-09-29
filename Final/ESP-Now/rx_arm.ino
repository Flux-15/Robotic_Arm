/*
 * RX ARM  (TX ESP32 --ESP-NOW--> this ESP32 --> PCA9685 --> servos)
 *
 * ESP-NOW packets carry exactly the same text lines your PC used to send over
 * USB serial, e.g.   S0:90,S1:45,S2:120,S3:90,S4:90,S5:50\n
 *
 * Both sources (ESP-NOW and USB serial, handy for debugging) feed the SAME
 * line assembler -> processLine(). If your existing firmware already has a
 * line parser, delete my parseCommand() and call YOUR function from
 * processLine() instead. Everything else here is transport only.
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ================== USER CONFIG ==================
#define WIFI_CHANNEL   1        // must match TX
#define SERIAL_BAUD    115200
#define NUM_SERVOS     6        // 5 joints + gripper
#define SDA_PIN        21
#define SCL_PIN        22
#define SERVO_FREQ     50
#define US_MIN         500      // pulse @0 deg   (tune to your servos)
#define US_MAX         2500     // pulse @180 deg
#define EMA_ALPHA      0.15f    // smoothing factor
#define DEADZONE_DEG   2.0f     // ignore changes smaller than this
#define UPDATE_MS      20       // 50 Hz servo update
#define LINK_TIMEOUT   1000     // ms without packets -> flag link lost
#define FEEDBACK_MS    1000     // heartbeat back to TX (0 = disabled)
// per-joint invert (true = 180 - angle)
bool invertJoint[NUM_SERVOS] = {false, false, false, false, false, false};
// =================================================

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

float targetDeg[NUM_SERVOS];
float currentDeg[NUM_SERVOS];
float lastAppliedTarget[NUM_SERVOS];

// ---------- ESP-NOW receive queue (callback -> loop) ----------
struct Packet { uint8_t len; char data[250]; };
static QueueHandle_t rxQueue;

static uint8_t txMac[6];
static bool    txKnown = false;
static uint32_t lastRxMs = 0;
static bool    linkLost = true;

#if defined(ESP_ARDUINO_VERSION) && ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  const uint8_t *mac = info->src_addr;
#else
void onRecv(const uint8_t *mac, const uint8_t *data, int len) {
#endif
  if (len <= 0 || len > 250) return;
  Packet p;
  p.len = len;
  memcpy(p.data, data, len);
  xQueueSend(rxQueue, &p, 0);          // drop if full (positions are idempotent)
  lastRxMs = millis();
  if (!txKnown) { memcpy(txMac, mac, 6); txKnown = true; }
}

// ---------- servo output ----------
void writeServo(uint8_t ch, float deg) {
  deg = constrain(deg, 0.0f, 180.0f);
  if (invertJoint[ch]) deg = 180.0f - deg;
  float us = US_MIN + (US_MAX - US_MIN) * (deg / 180.0f);
  pwm.writeMicroseconds(ch, (uint16_t)us);
}

// ---------- command parsing ----------
// Format: "S0:90,S1:45,...\n"  (any subset of channels is fine)
void parseCommand(char *line) {
  char *saveptr;
  for (char *tok = strtok_r(line, ",", &saveptr); tok; tok = strtok_r(NULL, ",", &saveptr)) {
    int ch, val;
    if (sscanf(tok, " S%d:%d", &ch, &val) == 2 && ch >= 0 && ch < NUM_SERVOS) {
      float v = constrain((float)val, 0.0f, 180.0f);
      if (fabsf(v - lastAppliedTarget[ch]) >= DEADZONE_DEG) {
        targetDeg[ch] = v;
        lastAppliedTarget[ch] = v;
      }
    }
  }
}

// One complete line arrived (from ESP-NOW OR USB serial)
void processLine(char *line) {
  if (line[0] == '#' || line[0] == '\0') return;   // ignore debug/empty
  parseCommand(line);        // <-- replace with YOUR existing handler if different
}

// ---------- shared line assembler ----------
static char   asmBuf[256];
static size_t asmLen = 0;

void feedChar(char c) {
  if (c == '\r') return;
  if (c == '\n') {
    asmBuf[asmLen] = '\0';
    processLine(asmBuf);
    asmLen = 0;
  } else if (asmLen < sizeof(asmBuf) - 1) {
    asmBuf[asmLen++] = c;
  } else {
    asmLen = 0;              // overflow, drop
  }
}

// ---------- optional feedback to TX/PC ----------
void sendFeedback(const char *msg) {
  if (!txKnown) return;
  if (!esp_now_is_peer_exist(txMac)) {      // register TX as a peer on first use
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, txMac, 6);
    peer.channel = WIFI_CHANNEL;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) return;
  }
  esp_now_send(txMac, (const uint8_t *)msg, strlen(msg));
}

void setup() {
  Serial.begin(SERIAL_BAUD);

  Wire.begin(SDA_PIN, SCL_PIN);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  for (int i = 0; i < NUM_SERVOS; i++) {
    targetDeg[i] = currentDeg[i] = lastAppliedTarget[i] = 90.0f;
    writeServo(i, 90.0f);
  }

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("#ESP-NOW init FAILED");
    while (true) delay(1000);
  }
  rxQueue = xQueueCreate(16, sizeof(Packet));
  esp_now_register_recv_cb(onRecv);

  Serial.print("#RX arm ready. My MAC: ");
  Serial.println(WiFi.macAddress());
}

void loop() {
  static uint32_t lastUpdate = 0, lastFb = 0;

  // 1) drain ESP-NOW packets
  Packet p;
  while (xQueueReceive(rxQueue, &p, 0) == pdTRUE) {
    for (uint8_t i = 0; i < p.len; i++) feedChar(p.data[i]);
  }

  // 2) USB serial still works for local debugging
  while (Serial.available()) feedChar((char)Serial.read());

  // 3) smooth servo update (EMA)
  if (millis() - lastUpdate >= UPDATE_MS) {
    lastUpdate = millis();
    for (int i = 0; i < NUM_SERVOS; i++) {
      currentDeg[i] += EMA_ALPHA * (targetDeg[i] - currentDeg[i]);
      writeServo(i, currentDeg[i]);
    }
  }

  // 4) link watchdog (arm simply holds last target)
  bool lost = (millis() - lastRxMs) > LINK_TIMEOUT;
  if (lost && !linkLost)  { Serial.println("#LINK LOST - holding position"); }
  if (!lost && linkLost)  { Serial.println("#LINK OK"); }
  linkLost = lost;

  // 5) heartbeat back to TX -> shows up on PC serial
  if (FEEDBACK_MS && txKnown && millis() - lastFb >= FEEDBACK_MS) {
    lastFb = millis();
    sendFeedback(linkLost ? "#RX_LINK_LOST\n" : "#RX_OK\n");
  }
}
