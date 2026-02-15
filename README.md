# NANOHOLD R.E.G.A | Robotic Enhanced Grip Assistant

### Operating Modes

#### 🟢 Automatic Mode
The system performs complete open-close cycles with sensor-based triggering:
1. **Object Detection** – VL53L4CD sensor detects object presence with 8-sample debounce
2. **Closing Phase** – Servo presses down until stall detection (Red LED illuminated)
3. **Opening Phase** – Servo retracts to up position after 5 second hold (Green LED illuminated)
4. **Completion** – Returns to idle state, ready for next detection

#### 👤 Manual Mode (Button Control)
User intervention takes priority with instant response:
- **Direct Control**: Left button (UP), Right button (DOWN)
- **8-Sample Debounce**: Prevents false triggers from noise
- **Emergency Stop**: Any button press during movement immediately halts operation
- **Single Action**: Each button press completes one movement before stopping

### Visual Indicators

| State | LED | Description |
|-------|-----|-------------|
| Opening | 🟢 Green Solid | Actively moving upward |
| Closing | 🔴 Red Solid | Actively pressing downward |
| Reset | ⚡ Fast Blink | Both buttons held 4-5 seconds |
| Idle | ⚫ Off | System idle or powered down |

### Power Management

| Mode | Timeout | Description |
|------|---------|-------------|
| **Active** | – | Normal operation, all systems enabled |
| **Power Save** | 1 min idle | LDO disabled, LED off, wake on button |
| **Shutdown** | 5 min idle | Full SYSTEMOFF with button wake |

### Safety & Protection

- **Emergency Reset**: Hold both buttons 5+ seconds for system reboot
- **Stall Detection**: Servo stops automatically at mechanical limit
- **Torque Disable**: Motor disabled after movement completion
- **Debounced Inputs**: 8-sample history for reliable button detection
- **Latch Mechanism**: Prevents duplicate command transmission

### Technical Specifications

| Component | Specification |
|-----------|---------------|
| **Microcontroller** | nRF52840 |
| **Servo** | Serial bus servo with position feedback |
| **Sensor** | VL53L4CD Time-of-Flight |
| **Power** | LDO-controlled, minimal idle consumption |
| **Indicators** | Bi-color LED (Red/Green) |

### 3D Model
[NANOHOLD R.E.G.A on MakerWorld](https://makerworld.com/en/models/2253633-nanohold-r-e-g-a)

---

*Robotic Enhanced Grip Assistant – Smart Press Control*
