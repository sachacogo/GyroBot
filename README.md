# GyroBot: A Self-Balancing Robot

[![Project Video](https://img.shields.io/badge/Watch%20the-Video-FF0000?style=for-the-badge&logo=youtube)](https://youtu.be/EcyiQm1V0ns)

A self-balancing robot built by repurposing hardware from a DrawBot project. This project served as a practical tutorial on PID control, sensor fusion, and embedded systems programming.

## Project Overview

The GyroBot stabilizes itself around its equilibrium point using an IMU to estimate its tilt angle and a PID controller to generate corrective motor commands.

### Key Concepts:
- **Sensor Fusion**: A Complementary Filter combines accelerometer and gyroscope data
- **PID Control**: Empirical tuning of a PID controller
- **PWM Motor Control**: Variable speed control on ESP32

## Hardware

- **Microcontroller**: NodeMCU ESP32
- **IMU Sensor**: LSM6DS3 (Accelerometer + Gyroscope)
- **Motors**: 2x N20 Geared DC motors (100 RPM)
- **Power**: 2x AA battery packs

## How It Works

1. ESP32 reads IMU data
2. Complementary filter calculates tilt angle
3. PID controller processes error
4. PWM signals drive motors to maintain balance

## Getting Started

### Prerequisites
- Arduino IDE with ESP32 support
- LSM6DS3 library

### Installation
1. Clone repository
2. Open GyroBot.ino in Arduino IDE
3. Install required library
4. Upload to ESP32

### PID Tuning
Tune Kp, Ki, Kd empirically: start with Kp, add Kd for damping, then small Ki for stability.

## License

MIT License - see LICENSE file for details.
