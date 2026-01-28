# NANOHOLD R.E.G.A | Robotic Enhanced Grip Assistant

## Operating Modes

### 🟢 Automatic Mode (Default)
The system automatically performs complete open-close cycles when ready:
1. **Opening Phase** – Moves upward until reaching target position (Green LED illuminated)
2. **Closing Phase** – Presses downward until detecting sufficient force (Red LED illuminated)
3. **Completion** – Returns to idle state after successful cycle

### 👤 Manual Mode (Button Override)
User intervention immediately takes priority:
- **Activation**: Any button press instantly switches control to manual
- **Direct Control**: Buttons command specific movements directly
- **Single Action**: System completes only the requested operation before stopping

## Visual Indicators
- **Solid Green LED**: Actively opening/moving upward
- **Solid Red LED**: Actively closing/pressing downward  
- **Fast Blinking LED**: Reset sequence active (both buttons held)
- **No LED**: System idle or power disabled

## Safety & Protection Features
- **Emergency Reset**: Hold both buttons for 5 seconds to trigger system reboot
- **Power Management**: Power automatically disable after 5 minute of inactivity
- **Command Protection**: Prevents duplicate movement commands with latch mechanism
- **Force Limiting**: Stops downward motion when predetermined pressure threshold reached

## Behavioral Notes
- Manual control always overrides automatic operation
- Each mode completes its current action before transitioning states
- System maintains last known position/force state between operations

## 3D Model
- [NANOHOLD R.E.G.A](https://makerworld.com/en/models/2253633-nanohold-r-e-g-a)
