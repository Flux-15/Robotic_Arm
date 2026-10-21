# Robotic Arm Motor Control Debugging Guide

**Last Updated:** October 21, 2026  
**Goal:** Systematically isolate whether motor control issues are hardware (wiring/power), driver (PCA9685), or software (firmware/protocol).

---

## Overview: The Debug Hierarchy

```
Individual Servo Test (GPIO PWM)
    ↓
PCA9685 Driver Test (all 16 channels)
    ↓
Full Arm Serial Test (with real commands)
    ↓
Troubleshooting Matrix (diagnosis)
    ↓
Perfboard PCB Design (hardened wiring)
```

**Key Insight:** Each level tests one layer. If step N fails, layer N is the problem. If step N passes but N+1 fails, the problem is between layers.

---

## Step 1: Individual Servo Test (Direct GPIO PWM, No PCA)

### Why This Step
- Tests servo and ESP32 GPIO in isolation
- Bypasses PCA9685 entirely
- No code complexity; just raw PWM

### Hardware Setup

```
ESP32 Pin 18 (GPIO18) ──→ Servo Signal (yellow/white)
ESP32 GND              ──→ Servo GND (black/brown)
ESP32 5V or ext supply ──→ Servo Power (red) 
                          [Use external 5-6V supply, NOT USB]
```

**Critical:** Use an external servo power supply (5-6V, minimum 2A). Do NOT power from USB alone.

### Code: Sweep Single Servo

```cpp
// File: single_servo_test.ino
// Compile for: ESP32 DevKit, 115200 baud

#define SERVO_PIN 18
#define PWM_FREQ 50      // 50 Hz for servo
#define PWM_CHANNEL 0
#define PWM_RESOLUTION 8 // 8-bit (0-255)

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Configure PWM
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(SERVO_PIN, PWM_CHANNEL);
  
  Serial.println("=== SINGLE SERVO TEST ===");
  Serial.println("GPIO18 PWM test starting...");
  Serial.println("Watch for smooth servo sweep 90° → 0° → 180° → 90°");
}

void loop() {
  // Servo positions in microseconds
  // 1000µs = 0° (min)
  // 1500µs = 90° (center)
  // 2000µs = 180° (max)
  
  int angles[] = {90, 0, 180, 90};
  
  for (int angle : angles) {
    int pwm_value = map(angle, 0, 180, 54, 204); // 1000µs to 2000µs at 50Hz, 8-bit
    
    Serial.print("Angle: ");
    Serial.print(angle);
    Serial.print("° → PWM: ");
    Serial.println(pwm_value);
    
    ledcWrite(PWM_CHANNEL, pwm_value);
    delay(2000); // Hold 2 seconds
  }
  
  Serial.println("---");
  delay(1000);
}
```

### Testing Procedure

1. **Upload** the sketch above to ESP32
2. **Power the servo** from external supply (connect GND together)
3. **Open Serial Monitor** (115200 baud)
4. **Observe:**
   - ✓ **Pass:** Servo smoothly rotates through 90°, 0°, 180°, 90°
   - ✗ **Fail — Buzzes only:** Power issue. Check supply voltage, current, common GND.
   - ✗ **Fail — No movement:** GPIO pin bad, servo bad, or wiring loose.

### Diagnosis

| Outcome | Likely Cause |
|---------|--------------|
| Servo moves smoothly | GPIO & servo are OK; proceed to Step 2 |
| Servo buzzes/twitches | Power supply inadequate or common GND missing |
| Servo locked in one position | Servo may be bad; try a different servo pin |
| No serial output | ESP32 not uploading; check USB cable and COM port |

---

## Step 2: PCA9685 Driver Test (All 16 Channels)

### Why This Step
- Tests I²C communication and the PCA9685 chip
- Tests each of the 16 servo channels
- Identifies bad channels or wiring

### Hardware Setup

```
ESP32 GPIO21 (SDA) ──→ PCA9685 SDA (yellow)
ESP32 GPIO22 (SCL) ──→ PCA9685 SCL (green)
ESP32 GND          ──→ PCA9685 GND
ESP32 5V or 3.3V   ──→ PCA9685 VCC (NOT servo power)

PCA9685 GND        ──→ Servo GND (all)
PCA9685 V+         ──→ External 5-6V supply (with bulk capacitor)

Connect one servo to PCA9685 Channel 0
```

**Important:** PCA9685 logic runs on 5V or 3.3V (NOT servo power). Servo power is separate on V+.

### Install Libraries

In Arduino IDE:
```
Sketch → Include Library → Manage Libraries
Search: "Adafruit PCA9685"
Install: Adafruit PCA9685 PWM Servo Driver
```

### Code: Test All 16 Channels

```cpp
// File: pca9685_test.ino
// Requires: Adafruit_PCA9685 library

#include <Wire.h>
#include <Adafruit_PCA9685.h>

Adafruit_PCA9685 pca;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("=== PCA9685 DRIVER TEST ===");
  
  // Initialize PCA9685 at default address 0x40
  if (!pca.begin()) {
    Serial.println("FAIL: PCA9685 not found on I²C bus!");
    Serial.println("Check SDA (GPIO21) and SCL (GPIO22) wiring.");
    while (1) delay(1000);
  }
  
  Serial.println("OK: PCA9685 found on I²C");
  
  // Set frequency to 50 Hz (servo standard)
  pca.setPWMFreq(50);
  delay(100);
  
  Serial.println("\nStarting channel sweep...");
  Serial.println("Channel | Min (0°)  | Center(90°) | Max(180°) | Result");
  Serial.println("--------|-----------|-------------|-----------|--------");
}

void loop() {
  // Test all 16 channels one at a time
  for (uint8_t channel = 0; channel < 16; channel++) {
    Serial.print("  ");
    Serial.print(channel);
    Serial.print("    | ");
    
    // Move to 0°
    pca.setPWM(channel, 0, 410);  // 1000µs pulse
    Serial.print("1000µs | ");
    delay(500);
    
    // Move to 90°
    pca.setPWM(channel, 0, 307);  // 1500µs pulse
    Serial.print("1500µs  | ");
    delay(500);
    
    // Move to 180°
    pca.setPWM(channel, 0, 204);  // 2000µs pulse
    Serial.print("2000µs | ");
    delay(500);
    
    // Return to center
    pca.setPWM(channel, 0, 307);
    Serial.println("OK");
    
    delay(500);
  }
  
  Serial.println("\n=== Cycle Complete ===\n");
  delay(2000);
}

// Utility: Microseconds to PCA9685 PWM value
// PCA9685 frequency = 50 Hz (20ms period)
// 12-bit resolution (0-4095)
// 1000µs = ~409 counts
// 1500µs = ~307 counts
// 2000µs = ~204 counts
```

### Testing Procedure

1. **Connect servo to Channel 0** of PCA9685
2. **Upload sketch** to ESP32
3. **Open Serial Monitor** (115200 baud)
4. **Watch the table:**
   - If servo moves on channel 0 → **Proceed to Step 3**
   - If servo never moves on channel 0 → **PCA not communicating**
   - If servo moves on some channels but not others → **Some channels are bad** (check solder on PCA9685, or the board itself)

### Diagnosis

| Symptom | Cause | Fix |
|---------|-------|-----|
| "PCA9685 not found" | I²C bus broken | Check SDA/SCL wiring; verify I²C address (use scanner sketch below) |
| Servo moves on channels 0-7 only | V+ power failing midway | Check bulk capacitor on V+ |
| Servo moves but twitches | PWM timing off | Use exact formula above; verify 50 Hz |
| No movement on any channel | PCA9685 chip bad | Try different board |

### Helper: I²C Address Scanner

If PCA9685 not found, run this to find the actual address:

```cpp
// File: i2c_scanner.ino

#include <Wire.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("I²C Address Scanner");
  Serial.println("Scanning...");
  
  byte count = 0;
  for (byte i = 8; i < 120; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      Serial.println(i, HEX);
      count++;
    }
  }
  
  Serial.print("Total devices: ");
  Serial.println(count);
}

void loop() {}
```

---

## Step 3: Full Robotic Arm Serial Test (Predefined Commands)

### Why This Step
- Tests firmware's serial parsing
- Tests IK calculations (if used)
- Tests joint mapping

### Hardware Setup

Same as Step 2, but connect all arm servos:
```
PCA9685 Channel 0 → Base servo
PCA9685 Channel 1 → Shoulder servo
PCA9685 Channel 2 → Elbow servo
PCA9685 Channel 3 → Wrist servo
PCA9685 Channel 4 → Roll servo
PCA9685 Channel 5 → Gripper servo
```

(Adjust according to your actual channel mapping.)

### Code: Robotic Arm Test with Serial Commands

```cpp
// File: robotic_arm_serial_test.ino
// Full arm test with predefined commands

#include <Wire.h>
#include <Adafruit_PCA9685.h>

Adafruit_PCA9685 pca;

// Channel mapping (ADJUST TO YOUR ARM)
#define CH_BASE      0
#define CH_SHOULDER  1
#define CH_ELBOW     2
#define CH_WRIST     3
#define CH_ROLL      4
#define CH_GRIPPER   5

// Servo limits (in degrees)
struct JointLimit {
  uint8_t minDeg;
  uint8_t maxDeg;
  uint8_t restDeg;  // Safe/neutral position
};

JointLimit limits[6] = {
  {0, 180, 90},    // Base
  {0, 120, 90},    // Shoulder
  {0, 150, 90},    // Elbow
  {0, 180, 90},    // Wrist
  {0, 180, 90},    // Roll
  {0, 180, 90}     // Gripper
};

// Servo names
const char* joint_names[6] = {"Base", "Shoulder", "Elbow", "Wrist", "Roll", "Gripper"};

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=== ROBOTIC ARM SERIAL TEST ===");
  Serial.println("Initializing PCA9685...");
  
  if (!pca.begin()) {
    Serial.println("ERROR: PCA9685 not found!");
    while (1) delay(1000);
  }
  
  pca.setPWMFreq(50);
  delay(100);
  
  Serial.println("PCA9685 initialized.");
  Serial.println("\nMove all joints to neutral (90°)...");
  
  // Move all to neutral
  for (int i = 0; i < 6; i++) {
    setJoint(i, limits[i].restDeg);
  }
  
  delay(2000);
  
  printMenu();
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input == "M") {
      printMenu();
    } 
    else if (input == "T") {
      testAllJoints();
    }
    else if (input.startsWith("J")) {
      // Format: "J0:45" → Set Joint 0 to 45°
      int colonIdx = input.indexOf(':');
      if (colonIdx > 0) {
        uint8_t joint = input.substring(1, colonIdx).toInt();
        uint8_t angle = input.substring(colonIdx + 1).toInt();
        
        if (joint < 6 && angle >= limits[joint].minDeg && angle <= limits[joint].maxDeg) {
          setJoint(joint, angle);
          Serial.print(joint_names[joint]);
          Serial.print(" set to ");
          Serial.print(angle);
          Serial.println("°");
        } else {
          Serial.println("ERROR: Invalid joint or angle out of range");
        }
      }
    }
    else if (input == "R") {
      Serial.println("Returning to neutral...");
      for (int i = 0; i < 6; i++) {
        setJoint(i, limits[i].restDeg);
      }
    }
    else if (input == "D") {
      debugJoints();
    }
    else {
      Serial.println("Unknown command. Type 'M' for menu.");
    }
  }
}

// Set a joint to a specific angle
void setJoint(uint8_t joint, uint8_t angle) {
  // Clamp angle to limits
  angle = constrain(angle, limits[joint].minDeg, limits[joint].maxDeg);
  
  // Convert angle to PWM value (0-180° → 204-410 counts at 50Hz, 12-bit)
  uint16_t pwmValue = map(angle, 0, 180, 204, 410);
  
  pca.setPWM(joint, 0, pwmValue);
}

// Test all joints sequentially
void testAllJoints() {
  Serial.println("\n=== TESTING ALL JOINTS ===");
  
  for (int joint = 0; joint < 6; joint++) {
    Serial.print("\nTesting ");
    Serial.println(joint_names[joint]);
    
    // Min → Max → Rest
    int positions[] = {limits[joint].minDeg, limits[joint].maxDeg, limits[joint].restDeg};
    
    for (int pos : positions) {
      setJoint(joint, pos);
      Serial.print("  → ");
      Serial.print(pos);
      Serial.println("°");
      delay(1500);
    }
  }
  
  Serial.println("\n=== TEST COMPLETE ===");
}

// Debug: Show current state of all joints
void debugJoints() {
  Serial.println("\n=== JOINT DEBUG INFO ===");
  for (int i = 0; i < 6; i++) {
    Serial.print(joint_names[i]);
    Serial.print(" | Min: ");
    Serial.print(limits[i].minDeg);
    Serial.print("° Max: ");
    Serial.print(limits[i].maxDeg);
    Serial.print("° Rest: ");
    Serial.print(limits[i].restDeg);
    Serial.println("°");
  }
  Serial.println("");
}

void printMenu() {
  Serial.println("\n╔════════════════════════════════════╗");
  Serial.println("║  ROBOTIC ARM CONTROL COMMANDS      ║");
  Serial.println("╠════════════════════════════════════╣");
  Serial.println("║ T    - Test all joints sequentially║");
  Serial.println("║ J[n]:[angle] - Move joint n to deg ║");
  Serial.println("║ R    - Return all to neutral (90°) ║");
  Serial.println("║ D    - Show joint debug info       ║");
  Serial.println("║ M    - Show this menu              ║");
  Serial.println("╚════════════════════════════════════╝");
  Serial.println("\nExamples:");
  Serial.println("  T          → Test sequence");
  Serial.println("  J0:45      → Move Base to 45°");
  Serial.println("  J1:120     → Move Shoulder to 120°");
  Serial.println("  J5:180     → Open Gripper");
}
```

### Testing Procedure

1. **Upload** the sketch above
2. **Open Serial Monitor** (115200 baud)
3. **Type 'M'** to see menu
4. **Run tests:**

   ```
   T          ← Test all joints in sequence
   ```

5. **Observe output:**
   - ✓ All 6 joints move smoothly → Proceed to **Troubleshooting Matrix**
   - ✗ Some joints don't move → Use **Troubleshooting Matrix** below
   - ✗ Gripper doesn't move → Check if **Channel 5** is connected; may be unimplemented in firmware

6. **Test individual joints:**

   ```
   J0:45      ← Base to 45°
   J1:120     ← Shoulder to 120°
   J2:90      ← Elbow to 90°
   J3:45      ← Wrist to 45°
   J4:180     ← Roll to 180°
   J5:90      ← Gripper to 90°
   ```

---

## Step 4: Troubleshooting Matrix

Use this table to diagnose after all tests above:

| Joint(s) Don't Move | Likely Cause | Test | Fix |
|---|---|---|---|
| **Shoulder only** | Power sag when gripper/wrist active | Run Step 3 test `J1:90` alone, then with all others | Add larger bulk capacitor (10000µF+) on servo supply; check gauge of power wires |
| **Wrist only** | Same as shoulder | Same | Same |
| **Gripper only** | Not implemented in firmware; no GUI control | Check firmware serial handler | Add gripper control to GUI or firmware |
| **All except Base** | V+ (servo power) supply failing | Check voltage with multimeter at PCA9685 V+ during operation | Upgrade supply to higher current rating (5A+ recommended) |
| **All except some** | Common ground missing between ESP32 and servo supply | Check GND connections | Add wire from servo supply GND to ESP32 GND |
| **Random twitching/jitter** | Jumper wires loose or noisy | Wiggle jumpers while watching servo | **→ BUILD PCB (Step 5)** |
| **Motors freeze/brownout reset** | Current spike when multiple servos move | Check ESP32 reset behavior in serial monitor | Add bulk capacitor AND reduce simultaneous motion in code |
| **PCA9685 channels 8-15 dead** | I²C address mismatch or channel enable bits | Run I²C scanner; check 0x40 vs 0x41 | Solder I²C address pins on PCA9685 to match address used in code |

---

## Step 5: Perfboard PCB Design (Replace Jumper Wires)

### Why Build a PCB

Jumper wires are the source of most "intermittent" failures:
- Loose connections cause dropouts
- Contact corrosion degrades signal
- EMI from long wires interferes with I²C and PWM
- Mechanical vibration breaks connections

A soldered perfboard replaces all of this with **solid, repeatable connections**.

### What You'll Build

```
Perfboard Layout:
┌─────────────────────────────────────────────┐
│ ESP32 DevKit                                │
│ (center-mounted)                            │
│                                             │
│  [PCA9685 breakout]  [Power connectors]    │
│  [I²C pull-ups]      [Servo supply]        │
│  [Servo headers]     [GND bus]             │
│  [Filter caps]       [ESP32 bypass cap]    │
└─────────────────────────────────────────────┘
```

### Materials Needed

- **Perfboard (generic):** 10×15 cm copper-clad (≈$2)
  - Or **Perfboard with bus rows** (easier routing)
- **Wire:** 22 AWG solid copper hookup wire (red, black, yellow)
- **Connectors:**
  - 2×3 header for ESP32 SDA/SCL (I²C)
  - 1×2 headers for servo power in/out
  - 1×16 female header strip (servo outputs; PCA9685 out)
- **Components:**
  - 2× 4.7kΩ resistors (I²C pull-ups; optional if on PCA9685 board)
  - 2× 100µF capacitors (bypass caps on PCA9685 VCC)
  - 1× 10000µF capacitor (bulk cap on servo supply)
  - 2× 1N4007 diodes (optional; reverse-polarity protection)
- **Soldering:** Iron (25W+), solder (60/40 or lead-free), flux, desoldering wick

### Step 5a: Design the Layout

**Critical Paths (short as possible):**
1. ESP32 GND ↔ PCA9685 GND ↔ Servo supply GND (thick wire, all in same area)
2. ESP32 GPIO21 (SDA) ↔ PCA9685 SDA
3. ESP32 GPIO22 (SCL) ↔ PCA9685 SCL
4. Servo supply in ↔ Bulk cap ↔ PCA9685 V+
5. PCA9685 outputs → Servo headers

**Layout Strategy:**

```
Perfboard (top view, showing 10×15 grid):

Row 1: [ESP32 GND BUS LINE - all 10 holes]
Row 2: [ESP32 VCC (3.3V) BUS LINE]
Row 3: (spacing)
Row 4: [ESP32 DevKit mounted here]
       SDA (GPIO21), SCL (GPIO22), GND, VCC exposed
Row 5-8: (wiring area)
Row 9: [PCA9685 breakout mounted here]
       VCC, GND, SDA, SCL, Signal pins exposed
Row 10: [Servo power input connectors]
        +5V, GND
Row 11: [Servo output headers]
        16× signal pins from PCA9685
Row 12-15: (Filter caps, pull-ups)
```

### Step 5b: Solder Procedure

**Phase 1: Solder buses and power**

1. Cut perfboard to size (or use pre-cut 10×15 cm)
2. Install power bus strips:
   - Strip 1 (Row 1): GND bus (all 10 holes in a line)
   - Strip 2 (Row 2): VCC bus (all 10 holes)
   - Use thick wire (18 AWG) and solder liberally
3. Mount ESP32 DevKit:
   - Pin headers to hold it ≈1cm above board
   - Solder all pins
4. Install PCA9685 breakout at Row 9
5. **Solder bulk capacitor (10000µF)** across servo power rails:
   - **+**: to servo V+ rail
   - **−**: to servo GND rail
   - Use thick wire; don't rely on fragile capacitor legs

**Phase 2: I²C and control signals**

6. I²C pull-ups (if not on PCA board):
   - 4.7kΩ resistor from SDA to 3.3V, solder to bus
   - 4.7kΩ resistor from SCL to 3.3V, solder to bus
7. Route I²C:
   - ESP32 GPIO21 (SDA) → 4.7k pull-up → PCA9685 SDA (use yellow wire)
   - ESP32 GPIO22 (SCL) → 4.7k pull-up → PCA9685 SCL (use yellow wire)
   - Use point-to-point soldering (solder joints on perfboard pads, not on components)

**Phase 3: Servo headers**

8. Install 16× servo output headers at Row 11:
   - Use 1×2 header for each servo (Signal, GND)
   - Or 1×16 single-row header + GND bus
9. Route signals from PCA9685 outputs to headers:
   - Channel 0 → Header 0
   - Channel 1 → Header 1
   - ... (all 16)
10. Every header shares the GND bus

**Phase 4: Verification**

11. **Continuity check:**
    - Multimeter in resistance mode
    - ESP32 GND pad ↔ Servo GND bus: **0Ω** (solid connection)
    - ESP32 VCC ↔ PCA9685 VCC: **short**, indicates good power path
    - SDA line: continuity from ESP32 → PCA9685
    - SCL line: continuity from ESP32 → PCA9685
12. **Voltage check (powered):**
    - Servo supply: **5-6V** across bulk cap
    - PCA9685 VCC: **5V or 3.3V** (whatever you connected)
    - ESP32 3.3V rail: **3.3V**

### Step 5c: Mounting Diagram

```
Perfboard Wiring Diagram (schematic):

        ┌─────────────────┐
        │   ESP32 DevKit  │
  GPIO21├────────┬───────┤ GND
  GPIO22├────────┤       │
    3.3V├────────┤       │
        └────┬───┴───────┘
             │
      ┌──[4.7kΩ]──┐
      ├──[4.7kΩ]──┤
      │            │
     GND          3.3V
      │            │
      ├────────────┼─────────────┐
      │            │             │
      │         ┌──────────────┐ │
      ├────────→│   PCA9685    │ │
      │         │   I²C: 0x40  │ │
      │         │              │ │
      │    ┌───→│ SDA, SCL     │ │
      │    │    │ GND, VCC(5V) │ │
      │    │    └──────────────┘ │
      │    │        │   │   │   │
      │    │        │  [10000µF] │ Bulk Cap
      │    │        │   │    │   │
   ┌──┴────┴─────────┴───┴────┴───┐ Servo Supply (5-6V external)
   │      GND BUS (main rail)     │
   └──────────────────────────────┘
      │
   ┌──┴─────────────────────────┐
   │  Servo Headers (1×16 or     │
   │  16× of 1×2)                │
   │                             │
   │ [0] [1] [2]...[15]          │
   │  │   │   │      │           │
   │  ├─[Sig]──┤      ├── ... Signals
   │  │                          │
   │  └──────────────────────────┴──→ GND (shared)
   └─────────────────────────────┘
```

### Step 5d: Assembly Checklist

- [ ] Perfboard cut and cleaned
- [ ] Power buses (GND, VCC) soldered (thick wire)
- [ ] ESP32 headers installed and all pins soldered
- [ ] PCA9685 headers installed and soldered
- [ ] I²C pull-ups (4.7kΩ) soldered
- [ ] Bulk capacitor (10000µF) soldered across servo supply rails
- [ ] Bypass capacitors (100µF) soldered on PCA9685 VCC
- [ ] All servo signal lines routed and soldered
- [ ] GND connections verified (continuity test all pairs)
- [ ] Power connections verified (no shorts; correct voltages)
- [ ] Test with Step 1 code (single servo GPIO18 → Servo on header 0)
- [ ] Test with Step 2 code (all PCA9685 channels)
- [ ] Test with Step 3 code (full arm)

### Step 5e: Soldering Tips

1. **Use flux liberally** — Helps solder flow and joints stick
2. **Tin wires first** — Strip ≈5mm, twist strands, coat with solder, then solder to board
3. **Use solder wick** for mistakes — Don't use solder sucker (damages perfboard pads)
4. **Keep joints small and shiny** — Dull/grainy = cold joint = unreliable
5. **Test as you go** — After every 5 joints, power on and verify with multimeter
6. **Hot-glue wires** after soldering to reduce mechanical stress

### Expected Result

After PCB is complete:
- No loose wires
- No noise/jitter in motor control
- Reliable operation even with mechanical vibration
- All servos respond consistently
- Shoulder and wrist work reliably when gripper is also moving

---

## Summary: What to Try First

1. **If only 3 motors work** (base, elbow, roll):
   - Run **Step 3 test** to confirm shoulder/wrist never get commands
   - Check firmware serial parser; likely omits those channels
   - Check IK solver; may return NaN for those joints

2. **If gripper has no GUI control**:
   - Add a slider/button to GUI HTML
   - Add gripper channel to the serial protocol
   - Upload updated firmware

3. **If random twitching** (jitter, dropouts):
   - Run **Step 1 + Step 2** to confirm hardware is solid
   - If solid, **build PCB** — jumper wires are the culprit

4. **If shoulder/wrist weak under load**:
   - Run **Step 2** test with all other joints idle
   - If shoulder works alone but fails when gripper active → **power supply too weak**
   - Upgrade to 5A supply and add 10000µF bulk capacitor

---

## Files Summary

| File | Purpose | When to Use |
|------|---------|-----------|
| `single_servo_test.ino` | GPIO18 PWM test | Step 1 — Confirm servo is alive |
| `pca9685_test.ino` | All 16 PCA channels | Step 2 — Confirm PCA9685 works |
| `robotic_arm_serial_test.ino` | Full arm with menu commands | Step 3 — Diagnose firmware bugs |
| `i2c_scanner.ino` | Find I²C devices | Troubleshooting only |
| Perfboard PCB layout | Soldered wiring | Step 5 — Replace jumper wires |

---

**Last Resort:** If all steps pass but arm still fails intermittently, the issue is **mechanical vibration loosening jumper connections**. Build the perfboard PCB. This is your answer.
