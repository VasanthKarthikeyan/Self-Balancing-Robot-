# Self-Balancing Robot (ESP32 + MPU6050 + TB6612FNG)

## Overview

This project is a **two-wheeled self-balancing robot** based on the classic **inverted pendulum system**. The robot continuously measures its tilt angle using an **MPU6050 IMU sensor** and applies real-time corrections through a **PID control algorithm** running on an **ESP32 microcontroller**.

By combining sensor fusion, motor control, embedded systems, and mechanical design, this project demonstrates practical implementation of real-time balancing robotics.

---

## Features

* Real-time self-balancing using PID control
* ESP32-based high-speed processing
* MPU6050 accelerometer + gyroscope sensor fusion
* Complementary filter for angle estimation
* TB6612FNG dual motor driver for efficient motor control
* PWM-based motor speed adjustment
* Custom 3D printed chassis design
* Battery-powered portable system
* Expandable for Bluetooth/Wi-Fi remote control

---

## Hardware Components

| Component              | Description                           |
| ---------------------- | ------------------------------------- |
| ESP32 DevKit V1        | Main controller board                 |
| MPU6050                | IMU sensor for tilt measurement       |
| TB6612FNG              | Dual H-Bridge motor driver            |
| N20 DC Gear Motors     | Wheel actuation                       |
| HW-133A / LM2596       | Buck converter for voltage regulation |
| 18650 Li-ion Batteries | Power source                          |
| 3D Printed Chassis     | Structural frame                      |

---

## System Architecture

### Workflow:

1. MPU6050 reads accelerometer and gyroscope data
2. Complementary filter estimates tilt angle
3. PID controller calculates correction
4. ESP32 generates PWM output
5. TB6612FNG drives motors
6. Wheels move to restore center of gravity

---

## Software Stack

* **Language:** C++ / Arduino Framework
* **Platform:** ESP32
* **Libraries:**

  * Wire.h
  * MPU6050 sensor communication
  * PWM motor control

---

## PID Control

The robot uses:

**PID Output = Kp × Error + Ki × Integral + Kd × Derivative**

### Parameters:

* **Kp** → Immediate correction
* **Ki** → Eliminates steady-state error
* **Kd** → Damping and stability

---

## Mechanical Design

* Fully custom **3D printed upper and lower chassis plates**
* Lightweight design for reduced motor load
* Optimized center of mass for improved balancing
* Modular mounting for electronics and batteries

---

## Challenges Faced

### Hardware:

* Short circuit in motor driver section
* Diagnosed using continuity testing
* Corrected wiring fault for proper motor functionality

### Software:

* Intermittent ESP32 COM port detection issues
* Flashing instability during code upload
* Required debugging of USB and board communication

---

## Future Improvements

* Wheel encoder integration
* Kalman filter implementation
* Bluetooth remote control
* Autonomous navigation
* LiDAR / ultrasonic obstacle detection
* Advanced control systems (LQR / adaptive PID)
* Mobile app integration

---

## Project Applications

* Robotics education
* Control systems experimentation
* Autonomous delivery systems
* Telepresence robots
* Embedded systems learning
* Advanced mechatronics projects

---



## Getting Started

### Requirements:

* Arduino IDE / PlatformIO
* ESP32 Board Package
* MPU6050 Library
* TB6612FNG Driver Wiring

### Steps:

1. Assemble chassis
2. Connect electronics as per schematic
3. Upload firmware to ESP32
4. Tune PID constants
5. Test balance carefully

---

## Results

* Successful motor actuation achieved
* Stable sensor fusion implemented
* Real-time balancing logic operational
* Mechanical prototype completed
* System serves as strong educational and engineering platform

---

## Author

**Vasanth Karthikeyan**
Mechatronics Engineering Student
Focused on robotics, embedded systems, and advanced prototyping.

---

## License

This project is open-source and available for educational and development purposes.

---

## Acknowledgment

This project represents practical integration of:

* Embedded systems
* Control theory
* Mechanical design
* Sensor fusion
* Real-time robotics engineering

It serves as a foundational step toward more advanced autonomous robotic systems.
