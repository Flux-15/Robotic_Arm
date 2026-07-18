/*
  5-DOF Robotic Arm — DUAL ARM, SHARED BOARD variant
  One ESP32 + one PCA9685 (16 channels) driving TWO arms:
    Arm A -> PCA9685 channels 0-5
    Arm B -> PCA9685 channels 6-11

  This is the same FK/IK model as arm_kinematics_control.ino, just
  duplicated per-arm (own link lengths, own servo calibration, own
  current/target angle state) and addressed over one serial port by
  prefixing every command with the arm letter.

  Use this file INSTEAD OF arm_kinematics_control.ino when both arms
  are wired to the same board. If your two arms are on two separate
  ESP32+PCA9685 sets, flash each board with the plain single-arm
  arm_kinematics_control.ino instead — the console GUI talks to each
  board over its own serial port in that case.

  SERIAL PROTOCOL (115200 baud, newline-terminated ASCII):
    <arm> J t1 t2 t3 t4 t5 grip   -> move that arm to joint angles (deg), grip 0-180
    <arm> P x y z psi grip        -> move that arm to Cartesian pose (mm, deg) via IK, elbow-up
    <arm> V x y z                 -> "vision" pose, fixed approach psi
    <arm> H                       -> home that arm
    <arm> G n                     -> set that arm's gripper opening, n = 0..180
    <arm> ?                       -> print that arm's current joint angles + pose

  <arm> is 'A' or 'B' (case-insensitive). Example: "A J 0 30 -40 -10 0 90"

  TODO before running on hardware:
    - MEASURE_YOUR_ARM: fill in LINK_A / LINK_B below (each arm can differ)
    - Fill in SERVO_CAL[] pulse/angle calibration for all 12 channels
    - If a shoulder/elbow is mounted mirrored on either arm, set that
      channel's `invert` to true — do NOT change the FK/IK math for this.
    - Confirm the channel map / offsets match your wiring
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// ---------------------------------------------------------------------
// LINK LENGTHS (mm) per arm — MEASURE_YOUR_ARM
// ---------------------------------------------------------------------
struct LinkConfig {
  float L0, A, B, L1, L2, L34;
};
LinkConfig LINK_A = { 75.0, 14.0, 12.0, 102.0, 130.0, 173.0 };
LinkConfig LINK_B = { 75.0, 14.0, 12.0, 102.0, 130.0, 173.0 };

// ---------------------------------------------------------------------
// CHANNEL MAP — Arm A on 0-5, Arm B on 6-11 (confirm against your wiring)
// ---------------------------------------------------------------------
const uint8_t OFFSET_A = 0;
const uint8_t OFFSET_B = 6;
// within each 6-channel bank: 0=base 1=shoulder 2=elbow 3=wrist 4=roll 5=grip

// ---------------------------------------------------------------------
// PER-SERVO CALIBRATION, 12 channels (0-5 = Arm A, 6-11 = Arm B)
// ---------------------------------------------------------------------
struct ServoCal {
  uint16_t pulseAt0;
  uint16_t pulseAt180;
  float angleMin;
  float angleMax;
  bool invert;
  float zeroOffset;
};

ServoCal SERVO_CAL[12] = {
  // ---- Arm A (channels 0-5) ----
  /* base    */ {102, 512, -150, 150, false, 0.0},
  /* shoulder*/ {102, 512,  -90, 120, true,  0.0},  // invert if mirrored on your build
  /* elbow   */ {102, 512, -130, 130, true,  0.0},  // invert if mirrored on your build
  /* wrist   */ {102, 512, -120, 120, false, 0.0},
  /* roll    */ {102, 512, -180, 180, false, 0.0},
  /* grip    */ {102, 512,    0, 180, false, 0.0},
  // ---- Arm B (channels 6-11) ----
  /* base    */ {102, 512, -150, 150, false, 0.0},
  /* shoulder*/ {102, 512,  -90, 120, false, 0.0},  // set per this arm's own mounting
  /* elbow   */ {102, 512, -130, 130, false, 0.0},  // set per this arm's own mounting
  /* wrist   */ {102, 512, -120, 120, false, 0.0},
  /* roll    */ {102, 512, -180, 180, false, 0.0},
  /* grip    */ {102, 512,    0, 180, false, 0.0},
};

// ---------------------------------------------------------------------
const float EMA_ALPHA = 0.15;
const float DEAD_ZONE_DEG = 0.15;
const unsigned long SERVO_UPDATE_MS = 20;

float currentAngle[12] = {0,30,-40,-10,0,90, 0,30,-40,-10,0,90};
float targetAngle[12]  = {0,30,-40,-10,0,90, 0,30,-40,-10,0,90};
unsigned long lastUpdate = 0;

// ---------------------------------------------------------------------
struct Pose { float Px, Py, Pz, psi; };

Pose forwardKinematics(float t1, float t2, float t3, float t4, LinkConfig &L) {
  float t1r = radians(t1), t2r = radians(t2), t3r = radians(t3), t4r = radians(t4);
  float sum23 = t2r + t3r, sum234 = t2r + t3r + t4r;
  float r = L.B + L.L1*cos(t2r) + L.L2*cos(sum23) + L.L34*cos(sum234);
  float z = L.L0 + L.A + L.L1*sin(t2r) + L.L2*sin(sum23) + L.L34*sin(sum234);
  Pose p; p.Px = r*cos(t1r); p.Py = r*sin(t1r); p.Pz = z; p.psi = t2+t3+t4;
  return p;
}

bool inverseKinematics(float Px, float Py, float Pz, float psiDeg, int elbowSign, LinkConfig &L, float out[4]) {
  float t1 = degrees(atan2(Py, Px));
  float rr = sqrt(Px*Px + Py*Py) - L.B;
  float zz = Pz - L.L0 - L.A;
  float psir = radians(psiDeg);
  float Wr = rr - L.L34*cos(psir);
  float Wz = zz - L.L34*sin(psir);
  float L1 = L.L1, L2 = L.L2;
  float D = (Wr*Wr + Wz*Wz - L1*L1 - L2*L2) / (2.0*L1*L2);
  bool reachable = (D >= -1.0 && D <= 1.0);
  if (D > 1.0) D = 1.0; if (D < -1.0) D = -1.0;
  float t3r = atan2(elbowSign*sqrt(1.0-D*D), D);
  float t2r = atan2(Wz, Wr) - atan2(L2*sin(t3r), L1+L2*cos(t3r));
  float t2 = degrees(t2r), t3 = degrees(t3r);
  float t4 = psiDeg - t2 - t3;
  out[0]=t1; out[1]=t2; out[2]=t3; out[3]=t4;
  return reachable;
}

// ---------------------------------------------------------------------
uint16_t angleToPulse(uint8_t ch, float angleDeg) {
  ServoCal &c = SERVO_CAL[ch];
  float a = constrain(angleDeg, c.angleMin, c.angleMax);
  float a180 = a - c.angleMin;
  float span = c.angleMax - c.angleMin;
  float frac = (span > 0.001) ? (a180/span) : 0.0;
  if (c.invert) frac = 1.0 - frac;
  return c.pulseAt0 + (uint16_t)(frac * (c.pulseAt180 - c.pulseAt0));
}

void writeServo(uint8_t ch, float angleDeg) {
  pwm.setPWM(ch, 0, angleToPulse(ch, angleDeg));
}

void setTargetsForArm(uint8_t off, float t1, float t2, float t3, float t4, float t5, float grip) {
  float v[6] = {t1,t2,t3,t4,t5,grip};
  for (int i = 0; i < 6; i++) {
    targetAngle[off+i] = constrain(v[i], SERVO_CAL[off+i].angleMin, SERVO_CAL[off+i].angleMax);
  }
}

void serviceServos() {
  unsigned long now = millis();
  if (now - lastUpdate < SERVO_UPDATE_MS) return;
  lastUpdate = now;
  for (int i = 0; i < 12; i++) {
    float diff = targetAngle[i] - currentAngle[i];
    if (fabs(diff) < DEAD_ZONE_DEG) currentAngle[i] = targetAngle[i];
    else currentAngle[i] += diff * EMA_ALPHA;
    writeServo(i, currentAngle[i]);
  }
}

// ---------------------------------------------------------------------
char lineBuf[96];
uint8_t lineLen = 0;

void handleLine(char *line) {
  char *p = line;
  while (*p == ' ') p++;

  char armCh = *p; p++;
  uint8_t off; LinkConfig *L;
  if (armCh == 'A' || armCh == 'a') { off = OFFSET_A; L = &LINK_A; }
  else if (armCh == 'B' || armCh == 'b') { off = OFFSET_B; L = &LINK_B; }
  else { Serial.println("ERR line must start with A or B (arm select)"); return; }

  while (*p == ' ') p++;
  char cmd = *p; p++;

  if (cmd == 'J' || cmd == 'j') {
    float v[6];
    for (int i = 0; i < 6; i++) v[i] = strtod(p, &p);
    setTargetsForArm(off, v[0], v[1], v[2], v[3], v[4], v[5]);
    Serial.print("OK "); Serial.print(armCh); Serial.println(" J");

  } else if (cmd == 'P' || cmd == 'p') {
    float x=strtod(p,&p), y=strtod(p,&p), z=strtod(p,&p), psi=strtod(p,&p), grip=strtod(p,&p);
    float out[4];
    bool ok = inverseKinematics(x, y, z, psi, 1, *L, out);
    setTargetsForArm(off, out[0], out[1], out[2], out[3], currentAngle[off+4], grip);
    Serial.print(ok ? "OK " : "WARN unreachable, clamped "); Serial.print(armCh); Serial.println(" P");

  } else if (cmd == 'V' || cmd == 'v') {
    const float APPROACH_PSI = -90.0;
    float x=strtod(p,&p), y=strtod(p,&p), z=strtod(p,&p);
    float out[4];
    bool ok = inverseKinematics(x, y, z, APPROACH_PSI, 1, *L, out);
    setTargetsForArm(off, out[0], out[1], out[2], out[3], currentAngle[off+4], currentAngle[off+5]);
    Serial.print(ok ? "OK " : "WARN unreachable, clamped "); Serial.print(armCh); Serial.println(" V");

  } else if (cmd == 'G' || cmd == 'g') {
    float g = strtod(p, &p);
    targetAngle[off+5] = constrain(g, SERVO_CAL[off+5].angleMin, SERVO_CAL[off+5].angleMax);
    Serial.print("OK "); Serial.print(armCh); Serial.println(" G");

  } else if (cmd == 'H' || cmd == 'h') {
    setTargetsForArm(off, 0, 30, -40, -10, 0, 90);
    Serial.print("OK "); Serial.print(armCh); Serial.println(" H");

  } else if (cmd == '?') {
    Pose pose = forwardKinematics(currentAngle[off+0], currentAngle[off+1], currentAngle[off+2], currentAngle[off+3], *L);
    Serial.print(armCh); Serial.print(" theta: ");
    for (int i = 0; i < 6; i++) { Serial.print(currentAngle[off+i]); Serial.print(' '); }
    Serial.print(" | pose: ");
    Serial.print(pose.Px); Serial.print(' '); Serial.print(pose.Py); Serial.print(' ');
    Serial.print(pose.Pz); Serial.print(' '); Serial.println(pose.psi);

  } else {
    Serial.println("ERR unknown cmd (use J/P/V/G/H/? after arm letter)");
  }
}

void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) { lineBuf[lineLen] = '\0'; handleLine(lineBuf); lineLen = 0; }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    }
  }
}

// ---------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(300);

  setTargetsForArm(OFFSET_A, 0, 30, -40, -10, 0, 90);
  setTargetsForArm(OFFSET_B, 0, 30, -40, -10, 0, 90);
  for (int i = 0; i < 12; i++) currentAngle[i] = targetAngle[i];
  for (int i = 0; i < 12; i++) writeServo(i, currentAngle[i]);

  Serial.println("Dual arm ready. Prefix every command with A or B, e.g. 'A J 0 30 -40 -10 0 90'");
}

void loop() {
  pollSerial();
  serviceServos();
}
