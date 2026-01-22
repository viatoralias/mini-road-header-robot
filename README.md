# Mini Road-Header Robot Project

A compact robotic system for autonomous small-scale tunnel boring in maintenance and electrical infrastructure applications.

## 📋 Project Description

This project implements a robotic system capable of:
- Autonomous mobility using 4 DC motors
- Precision arm control with 4 servo motors
- Boring operations with a high-torque brushless motor
- Advanced state estimation using Kalman filter

## 🛠️ Hardware Components

- **Controller**: Wemos D1 R32 (ESP32-based)
- **Shield**: Custom motor/servo controller (PCA9685 + motor drivers)
- **Actuators**: 
  - 4× DC motors for mobility (12V)
  - 4× MG996R servos for robotic arm
  - 1× Brushless motor for boring
- **Power**: 12V 4200mAh LiPo battery with buck converter

## 💻 Software & Code

### Getting Started

#### Option A: Arduino IDE
1. Open `firmware/full_control.ino`
2. Install required libraries:
   - Adafruit PWM Servo Driver Library
   - Adafruit BusIO
3. Select board: "Wemos D1 R32"
4. Upload to ESP32

#### Option B: PlatformIO (VSCode)
Use the project in `firmware/platformio_project/`

### Main Features
- Integrated control of all 4 DC motors and 5 servos
- I2C communication with PCA9685 servo driver
- Sequential servo sweeping for demonstration
- Continuous motor operation

## 📚 Documentation

- **Full Report**: [report.pdf](RoadHeader_Project_Benachou_Report.pdf) - Complete project documentation
- **Presentation**: [presentation.pdf](Autonomous-Mini-Road-Header.pdf) - Project overview slides

## 🔧 Key Technical Aspects

1. **Control System**: ESP32-based real-time control
2. **State Estimation**: Kalman filter implementation for arm positioning
3. **Power Management**: Separate power rails for motors and electronics
4. **Communication**: I2C-based servo control via PCA9685
