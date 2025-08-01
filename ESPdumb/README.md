# ESPdumb - ESP32 WiFi Servo Controller for Quadruped Robot

## Overview
This project allows you to control a quadruped robot's servos over WiFi using an ESP32 and a PCA9685 16-channel servo driver. The ESP32 receives UDP packets from a laptop and moves the servos accordingly. A Python script on the laptop sends commands for walking and rotating the robot.

## Hardware Connections

### ESP32 <-> PCA9685
- **SDA** (ESP32 GPIO 21)  → **SDA** (PCA9685)
- **SCL** (ESP32 GPIO 22)  → **SCL** (PCA9685)
- **VCC** (PCA9685)        → **3.3V or 5V** (match servo power)
- **GND** (PCA9685)        → **GND** (ESP32 and servo power supply)
- **V+** (PCA9685)         → **Servo power supply** (typically 5-6V)

> **Note:** Connect all grounds together (ESP32, PCA9685, and servo power supply).

### Servos
- Connect each servo's signal wire to the appropriate PCA9685 channel (see mapping below).
- Power servos from the external supply via PCA9685 V+ and GND.

### Servo Channel Mapping
| Leg          | Hip | Knee | Calf |
|--------------|-----|------|------|
| Front Left   |  4  |  3   |  2   |
| Front Right  |  8  |  9   | 10   |
| Hind Right   | 14  | 12   | 11   |
| Hind Left    |  7  |  6   |  5   |

## Software Setup

### ESP32
1. Flash `ESPdumb.ino` to your ESP32 using Arduino IDE or PlatformIO.
2. Install the following libraries:
   - `Adafruit PWM Servo Driver` (Adafruit_PWMServoDriver)
   - `WiFi` (built-in for ESP32)
3. Edit `ESPdumb.ino` to set your WiFi SSID and password.
4. Power the ESP32 and servos.

### Laptop Controller
1. Edit `laptop_controller.py` to set the correct ESP32 IP address (`ESP_IP`).
2. Run the script with Python 3: `python laptop_controller.py`
3. Use the following controls:
   - `w` = walk forward
   - `s` = walk backward
   - `a` = rotate left
   - `d` = rotate right
   - `q` = quit

## Notes
- The walking pattern is a diagonal crawl: three legs on the ground, one leg moves at a time.
- Make sure your servo power supply can handle the current for all servos.
- Test with servos disconnected first to verify communication.

---

**Author:** surya-d-naidu
