# Wireless Robotic Arm over ESP-NOW

Replaces the USB-serial cable between the PC and the arm with an ESP-NOW link. Nothing else in the system changes.

```
BEFORE:  PC ──USB serial──────────────────────────► Arm ESP32 ──► PCA9685 ──► servos

AFTER:   PC ──USB──► TX ESP32 ~~ESP-NOW~~► RX ESP32 (on arm) ──► PCA9685 ──► servos
                     (bridge)               (arm firmware)
```

- **TX ESP32** is a *transparent bridge*: each line the PC sends over serial becomes one ESP-NOW packet.
- **RX ESP32** receives those lines and feeds them into the same line parser the arm already used.
- **PC code / browser console is unchanged**. Just pick the **TX board's** COM port at the same baud rate.

## Folder layout

```
espnow/
├── get_mac/get_mac.ino      # prints a board's MAC address
├── tx_bridge/tx_bridge.ino  # flash on the ESP32 plugged into the PC
├── rx_arm/rx_arm.ino        # flash on the ESP32 on the arm
└── README.md
```

## Hardware

| Item | Notes |
|---|---|
| 2 × ESP32 dev boards | one TX (PC side), one RX (arm side) |
| PCA9685 + servos | unchanged, on the RX board (SDA=21, SCL=22 by default) |
| Power | RX ESP32 on its own stable 5 V supply, servos on a separate rail, common GND |

> **Power tip:** the Wi-Fi radio adds current spikes. Put a 470–1000 µF capacitor near the RX ESP32's 5 V/3V3 input and keep servo power off the ESP32 supply, or you'll get brown-out resets.

## Software prerequisites

1. Arduino IDE 2.x
2. **ESP32 board package** by Espressif (Boards Manager). Both core 2.x and 3.x are supported by these sketches.
3. Library: **Adafruit PWM Servo Driver Library** (Library Manager).

## Step-by-step

### Step 1: Get both MAC addresses
1. Open `get_mac/get_mac.ino`, flash it to the **RX** board, open Serial Monitor at 115200.
2. Write down the MAC, e.g. `24:6F:28:AB:CD:EF`. Label the board "RX".
3. Repeat for the **TX** board (optional, but useful for debugging).

### Step 2: Configure and flash the TX bridge
1. Open `tx_bridge/tx_bridge.ino`.
2. Put the RX MAC into `RX_MAC`, converting each byte to `0x..`:
   ```cpp
   uint8_t RX_MAC[6] = {0x24, 0x6F, 0x28, 0xAB, 0xCD, 0xEF};
   ```
3. Check `SERIAL_BAUD` matches what your PC software uses (default 115200).
4. Select your ESP32 board, select the TX board's port, upload.

### Step 3: Merge the receiver into your arm firmware
Open `rx_arm/rx_arm.ino`. It is a complete working arm firmware (PCA9685, EMA smoothing 0.15, 2° dead-zone, per-joint invert, 50 Hz update). Two options:

- **A. Use it as is.** Set `NUM_SERVOS`, pins, `US_MIN/US_MAX`, `invertJoint[]`, then upload.
- **B. Keep your existing arm firmware.** Copy over only the transport parts:
  - the `Packet` struct, `rxQueue`, and `onRecv()`
  - the WiFi/ESP-NOW block from `setup()`
  - the queue-drain loop at the top of `loop()`
  - `feedChar()` (the shared line assembler)
  - In `processLine()`, call **your** existing command handler instead of `parseCommand()`.

Your USB-serial input still works on the RX board, so you can keep testing over a cable.

### Step 4: Upload to the RX board
Select the RX board's port and upload. Serial Monitor should show `#RX arm ready. My MAC: ...`.

### Step 5: Bench test with no arm attached
1. Power the RX board (USB is fine for now).
2. Plug the TX board into the PC and open a serial monitor at 115200.
3. Type a line and press Enter:
   ```
   S0:120,S1:90,S2:90,S3:90,S4:90,S5:50
   ```
4. On the RX Serial Monitor you should see no errors, and after ~1 s the TX monitor should show `#RX_OK` heartbeats and `#TX ok=… fail=0`.

### Step 6: Run the real system
1. Power the arm (servo rail + RX ESP32).
2. Close the serial monitor. Only one program may hold the COM port.
3. Start your normal PC software (Python script / Web Serial console) and select the **TX board's COM port**.
4. Use it exactly as before. The arm now moves wirelessly.

## Behaviour and design notes

| Feature | Detail |
|---|---|
| Channel | Fixed to `WIFI_CHANNEL 1` on both boards (must match) |
| Framing | One text line = one packet (max ~240 bytes) |
| Debug lines | Board messages start with `#`. Make sure your PC parser ignores lines starting with `#`, or set `FEEDBACK_MS 0` and remove prints if it can't. |
| Feedback | RX sends `#RX_OK` / `#RX_LINK_LOST` once per second; TX forwards it to the PC |
| Link loss | If no packets for 1 s, RX holds its last position and prints `#LINK LOST` |
| Delivery | Unicast ESP-NOW has MAC-level ACK and retry. Position streams are idempotent, so a dropped packet is harmless. |
| Range | ~50–100 m line of sight, typically 10–30 m indoors |
| Latency | A few ms, well below the serial stream rate |

### Dual-arm / multiple receivers
Set `#define USE_BROADCAST true` in `tx_bridge.ino`. Every RX board in range gets every line (no ACK/retry in broadcast). If the two arms need different commands, use different channel labels in the line protocol (e.g. `A:` / `B:` prefixes) and filter in `processLine()`.

## Troubleshooting

| Symptom | Fix |
|---|---|
| `#TX ok=0 fail=N` | Wrong `RX_MAC`, RX not powered, or channel mismatch |
| Compile error in callback signature | Update the board package; the sketches guard for core 2.x and 3.x, but very old cores may differ |
| Arm jitters or resets | Separate servo power, add bulk capacitor at the ESP32, common GND |
| Nothing on PC after connecting | Wrong COM port. Use the **TX** board's, and close other serial programs |
| Arm moves slowly or lags | Raise `EMA_ALPHA` (e.g. 0.25) and lower `UPDATE_MS` |
| PC parser complains about `#` lines | Filter them in the PC code, or set `FEEDBACK_MS 0` and remove the 5 s stats print in the TX loop |
| Random dropouts | Keep the antennas clear of metal and away from a busy 2.4 GHz router; try another `WIFI_CHANNEL` (1, 6 or 11) on **both** boards |

## Optional upgrades
- **Encryption:** set `peer.encrypt = true` and a shared 16-byte PMK/LMK on both sides.
- **Binary protocol:** replace text lines with a packed `struct { uint8_t angles[6]; }` for smaller packets.
- **Emergency stop:** have the PC send `STOP\n` and handle it in `processLine()`.
