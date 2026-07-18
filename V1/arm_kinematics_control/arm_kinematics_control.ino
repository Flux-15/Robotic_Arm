/*
  5-DOF Robotic Arm — Kinematics-driven control
  ESP32 + PCA9685 + DS3115MG servos

  Implements the same decoupled geometric FK/IK model used in
  arm_kinematics_gui.html, derived from the DH parameters in:
  Ibrahim & Ali, "Inverse Kinematic Analysis For A 5 DOF Robotic Arm
  Using Deep Neural Network," IJES vol 11, no 18s, 2025.

  NOTE ON DEVIATION FROM THE PAPER:
  The paper solves IK with a trained DNN. On an ESP32, for a 4-variable
  decoupled chain like this one, a closed-form geometric solution (below)
  is exact, deterministic, and far cheaper than running inference on
  a microcontroller — so that's what this firmware uses. The geometry
  mirrors the paper's own decoupling insight (their eq. 11: psi =
  theta2+theta3+theta4), just solved analytically instead of learned.

  SERIAL PROTOCOL (115200 baud, newline-terminated ASCII):
    J t1 t2 t3 t4 t5 grip     -> move directly to joint angles (deg), grip 0-180
    P x y z psi grip          -> move to Cartesian pose (mm, deg) via IK, elbow-up
    V x y z                   -> "vision" pose: same as P with fixed approach psi
                                   and current gripper state (for CV/AprilTag hookup)
    H                         -> go to home pose
    G n                       -> set gripper opening, n = 0 (closed) .. 180 (open)
    ?                         -> print current joint angles + last commanded pose

  TODO before running on hardware:
    - MEASURE_YOUR_ARM: fill in LINK_* below from your physical arm
    - Fill in SERVO_CAL[] pulse/angle calibration per channel (from your
      existing calibration routine)
    - Confirm CHANNEL map matches your PCA9685 wiring
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// ---------------------------------------------------------------------
// LINK LENGTHS (mm) — MEASURE_YOUR_ARM: replace with your physical arm's
// dimensions. Defaults below are the paper's DH values as a starting point.
// ---------------------------------------------------------------------
struct LinkConfig {
  float L0;   // base height, ground to shoulder yaw axis
  float A;    // shoulder vertical offset
  float B;    // shoulder horizontal offset
  float L1;   // upper arm (shoulder to elbow)
  float L2;   // forearm (elbow to wrist)
  float L34;  // wrist to gripper tip (L3+L4 combined)
};
LinkConfig LINK = { 75.0, 14.0, 12.0, 102.0, 130.0, 173.0 };

// ---------------------------------------------------------------------
// PCA9685 CHANNEL MAP — confirm against your wiring
// ---------------------------------------------------------------------
const uint8_t CH_BASE   = 0;  // theta1
const uint8_t CH_SHLDR  = 1;  // theta2
const uint8_t CH_ELBOW  = 2;  // theta3
const uint8_t CH_WRIST  = 3;  // theta4
const uint8_t CH_ROLL   = 4;  // theta5
const uint8_t CH_GRIP   = 5;  // gripper open/close

// ---------------------------------------------------------------------
// PER-SERVO CALIBRATION — replace with your measured pulse counts.
// pulseAt0 / pulseAt180 are PCA9685 tick counts (0-4095 @ 50Hz) at the
// two ends of that servo's travel; angleMin/angleMax clamp commanded
// angles to what's mechanically safe for that joint.
// ---------------------------------------------------------------------
struct ServoCal {
  uint16_t pulseAt0;
  uint16_t pulseAt180;
  float angleMin;
  float angleMax;
  bool invert;
  float zeroOffset;   // kinematic angle that maps to this servo's "90 deg" center
};

ServoCal SERVO_CAL[6] = {
  /* CH_BASE  */ {102, 512, -150, 150, false, 0.0},
  /* CH_SHLDR */ {102, 512,  -90, 120, false, 0.0},
  /* CH_ELBOW */ {102, 512, -130, 130, false, 0.0},
  /* CH_WRIST */ {102, 512, -120, 120, false, 0.0},
  /* CH_ROLL  */ {102, 512, -180, 180, false, 0.0},
  /* CH_GRIP  */ {102, 512,    0, 180, false, 0.0},
};

// ---------------------------------------------------------------------
// Smoothing — matches the EMA + non-blocking ramp approach from prior
// firmware revisions on this arm
// ---------------------------------------------------------------------
const float EMA_ALPHA = 0.15;
const float DEAD_ZONE_DEG = 0.15;
const unsigned long SERVO_UPDATE_MS = 20;

float currentAngle[6] = {0, 30, -40, -10, 0, 90};
float targetAngle[6]  = {0, 30, -40, -10, 0, 90};
unsigned long lastUpdate = 0;

// ---------------------------------------------------------------------
// FK / IK — mirrors arm_kinematics_gui.html
// ---------------------------------------------------------------------
struct Pose { float Px, Py, Pz, psi; };

Pose forwardKinematics(float t1, float t2, float t3, float t4) {
  float t1r = radians(t1), t2r = radians(t2), t3r = radians(t3), t4r = radians(t4);
  float sum23 = t2r + t3r;
  float sum234 = t2r + t3r + t4r;
  float r = LINK.B + LINK.L1 * cos(t2r) + LINK.L2 * cos(sum23) + LINK.L34 * cos(sum234);
  float z = LINK.L0 + LINK.A + LINK.L1 * sin(t2r) + LINK.L2 * sin(sum23) + LINK.L34 * sin(sum234);
  Pose p;
  p.Px = r * cos(t1r);
  p.Py = r * sin(t1r);
  p.Pz = z;
  p.psi = t2 + t3 + t4;
  return p;
} 

// Returns true if solvable (target within reach). Writes result into out[4]: t1,t2,t3,t4
bool inverseKinematics(float Px, float Py, float Pz, float psiDeg, int elbowSign, float out[4]) {
  float t1 = degrees(atan2(Py, Px));
  float rr = sqrt(Px * Px + Py * Py) - LINK.B;
  float zz = Pz - LINK.L0 - LINK.A;
  float psir = radians(psiDeg);
  float Wr = rr - LINK.L34 * cos(psir);
  float Wz = zz - LINK.L34 * sin(psir);

  float L1 = LINK.L1, L2 = LINK.L2;
  float Dnum = (Wr * Wr + Wz * Wz - L1 * L1 - L2 * L2);
  float Dden = (2.0 * L1 * L2);
  float D = Dnum / Dden;

  bool reachable = (D >= -1.0 && D <= 1.0);
  if (D > 1.0) D = 1.0;
  if (D < -1.0) D = -1.0;

  float t3r = atan2(elbowSign * sqrt(1.0 - D * D), D);
  float t2r = atan2(Wz, Wr) - atan2(L2 * sin(t3r), L1 + L2 * cos(t3r));
  float t2 = degrees(t2r);
  float t3 = degrees(t3r);
  float t4 = psiDeg - t2 - t3;

  out[0] = t1; out[1] = t2; out[2] = t3; out[3] = t4;
  return reachable;
}

// ---------------------------------------------------------------------
// Servo output
// ---------------------------------------------------------------------
uint16_t angleToPulse(uint8_t ch, float angleDeg) {
  ServoCal &c = SERVO_CAL[ch];
  float a = constrain(angleDeg, c.angleMin, c.angleMax);
  float a180 = a - c.angleMin;               // shift into 0..(max-min)
  float span = c.angleMax - c.angleMin;
  float frac = (span > 0.001) ? (a180 / span) : 0.0;
  if (c.invert) frac = 1.0 - frac;
  uint16_t pulse = c.pulseAt0 + (uint16_t)(frac * (c.pulseAt180 - c.pulseAt0));
  return pulse;
}

void writeServo(uint8_t ch, float angleDeg) {
  pwm.setPWM(ch, 0, angleToPulse(ch, angleDeg));
}

void setTargets(float t1, float t2, float t3, float t4, float t5, float grip) {
  targetAngle[0] = constrain(t1, SERVO_CAL[0].angleMin, SERVO_CAL[0].angleMax);
  targetAngle[1] = constrain(t2, SERVO_CAL[1].angleMin, SERVO_CAL[1].angleMax);
  targetAngle[2] = constrain(t3, SERVO_CAL[2].angleMin, SERVO_CAL[2].angleMax);
  targetAngle[3] = constrain(t4, SERVO_CAL[3].angleMin, SERVO_CAL[3].angleMax);
  targetAngle[4] = constrain(t5, SERVO_CAL[4].angleMin, SERVO_CAL[4].angleMax);
  targetAngle[5] = constrain(grip, SERVO_CAL[5].angleMin, SERVO_CAL[5].angleMax);
}

void serviceServos() {
  unsigned long now = millis();
  if (now - lastUpdate < SERVO_UPDATE_MS) return;
  lastUpdate = now;
  for (int i = 0; i < 6; i++) {
    float diff = targetAngle[i] - currentAngle[i];
    if (fabs(diff) < DEAD_ZONE_DEG) {
      currentAngle[i] = targetAngle[i];
    } else {
      currentAngle[i] += diff * EMA_ALPHA;
    }
    writeServo(i, currentAngle[i]);
  }
}

// ---------------------------------------------------------------------
// Serial command parsing — char buffer + strtod, no String() allocations
// ---------------------------------------------------------------------
char lineBuf[96];
uint8_t lineLen = 0;

void handleLine(char *line) {
  char *p = line;
  while (*p == ' ') p++;
  char cmd = *p;
  p++;

  if (cmd == 'J' || cmd == 'j') {
    float v[6];
    for (int i = 0; i < 6; i++) v[i] = strtod(p, &p);
    setTargets(v[0], v[1], v[2], v[3], v[4], v[5]);
    Serial.println("OK J");

  } else if (cmd == 'P' || cmd == 'p') {
    float x = strtod(p, &p);
    float y = strtod(p, &p);
    float z = strtod(p, &p);
    float psi = strtod(p, &p);
    float grip = strtod(p, &p);
    float out[4];
    bool ok = inverseKinematics(x, y, z, psi, 1, out);
    setTargets(out[0], out[1], out[2], out[3], currentAngle[4], grip);
    Serial.print(ok ? "OK P " : "WARN unreachable, clamped P ");
    Serial.print(out[0]); Serial.print(' ');
    Serial.print(out[1]); Serial.print(' ');
    Serial.print(out[2]); Serial.print(' ');
    Serial.println(out[3]);

  } else if (cmd == 'V' || cmd == 'v') {
    // Vision/CV pose command: fixed top-down approach angle.
    // Adjust APPROACH_PSI to match how you want the gripper oriented
    // for pick-and-place once AprilTag/CV supplies x,y,z.
    const float APPROACH_PSI = -90.0;
    float x = strtod(p, &p);
    float y = strtod(p, &p);
    float z = strtod(p, &p);
    float out[4];
    bool ok = inverseKinematics(x, y, z, APPROACH_PSI, 1, out);
    setTargets(out[0], out[1], out[2], out[3], currentAngle[4], currentAngle[5]);
    Serial.println(ok ? "OK V" : "WARN unreachable, clamped V");

  } else if (cmd == 'G' || cmd == 'g') {
    float g = strtod(p, &p);
    targetAngle[5] = constrain(g, SERVO_CAL[5].angleMin, SERVO_CAL[5].angleMax);
    Serial.println("OK G");

  } else if (cmd == 'H' || cmd == 'h') {
    setTargets(0, 30, -40, -10, 0, 90);
    Serial.println("OK H");

  } else if (cmd == '?') {
    Pose pose = forwardKinematics(currentAngle[0], currentAngle[1], currentAngle[2], currentAngle[3]);
    Serial.print("theta: ");
    for (int i = 0; i < 6; i++) { Serial.print(currentAngle[i]); Serial.print(' '); }
    Serial.print(" | pose: ");
    Serial.print(pose.Px); Serial.print(' ');
    Serial.print(pose.Py); Serial.print(' ');
    Serial.print(pose.Pz); Serial.print(' ');
    Serial.println(pose.psi);

  } else {
    Serial.println("ERR unknown cmd (use J/P/V/G/H/?)");
  }
}

void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        handleLine(lineBuf);
        lineLen = 0;
      }
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

  // Home pose on boot
  setTargets(0, 30, -40, -10, 0, 90);
  for (int i = 0; i < 6; i++) currentAngle[i] = targetAngle[i];
  for (int i = 0; i < 6; i++) writeServo(i, currentAngle[i]);

  Serial.println("Arm ready. Commands: J t1 t2 t3 t4 t5 grip | P x y z psi grip | V x y z | G n | H | ?");
}

void loop() {
  pollSerial();
  serviceServos();
}
