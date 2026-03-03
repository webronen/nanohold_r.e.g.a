# NANOHOLD R.E.G.A | Robotic Enhanced Grip Assistant

### Operating Modes

#### 🟢 Automatic Mode
The system performs complete open-close cycles with sensor-based triggering:
1. **Object Detection** – VL53L4CD sensor detects object presence with 8-sample debounce
2. **Closing Phase** – Servo presses down until stall detection, holds for 1 second, then disables torque (🔴 Red LED)
3. **Opening Phase** – Servo retracts to up position and disables torque (🟢 Green LED)
4. **Completion** – Returns to idle state, ready for next detection

#### 👤 Manual Mode (Button Control)
User intervention takes priority with instant response:
- **Direct Control**: Left button (UP), Right button (DOWN)
- **8-Sample Debounce**: Prevents false triggers from noise
- **Single Action**: Each button press completes one movement before stopping

### Serial Remote Control

Control the system via serial terminal (115200 baud):

| Command | Action |
|---------|--------|
| `+` | Move UP one step |
| `-` | Move DOWN one step |
| `r` | Trigger system reset |
| `s` | Print JSON state |
| `?` | Show help menu |

**JSON Response:**
{"mode":1,"step":2,"power":1,"distance_mm":150,"time_us":12345678}

### Visual Indicators

| State | LED | Description |
|-------|-----|-------------|
| Press Open | 🟢 Green | Servo in up position, press open |
| Press Closed | 🔴 Red | Servo in down position, press closed |
| Reset | ⚡ Amber Blink | Both buttons held 4-5 seconds |
| Idle | ⚫ Off | System idle or powered down |

*Bi-color LED with tri-state control using series configuration (anode of one LED connected to cathode of the other)*

### Power Management

| Mode | Timeout | Description |
|------|---------|-------------|
| **Active** | – | Normal operation, all systems enabled |
| **Power Save** | 1 min idle | LDO disabled, LED off, wake on button |
| **Shutdown** | 5 min idle | Full SYSTEMOFF with button wake |

### Safety & Protection

- **Emergency Reset**: Hold both buttons 5+ seconds for system reboot
- **Stall Detection**: Servo stops automatically at mechanical limit
- **Torque Disable**: Motor disabled 1 second after stall or movement completion
- **Debounced Inputs**: 8-sample history for reliable button detection
- **Latch Mechanism**: Prevents duplicate command transmission
- **Remote Commands**: Direct control via serial interface

### Technical Specifications

| Component | Specification |
|-----------|---------------|
| **Microcontroller** | Supermini nRF52840 |
| **Servo** | Feetech SCS0009 Serial bus servo with position feedback |
| **Sensor** | SATEL-VL53L4CD Time-of-Flight |
| **Power** | LDO-controlled, minimal idle consumption |
| **LED** | Bi-color, series-connected with tri-state control |
| **Half-Duplex Conversion** | 1N5711 diode + 4.7kΩ pullup |

### 3D Model
[NANOHOLD R.E.G.A on MakerWorld](https://makerworld.com/en/models/2253633-nanohold-r-e-g-a)

---

*Robotic Enhanced Grip Assistant – Smart Press Control with Serial Remote*
