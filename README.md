# Line-Following HiL Robot

This project implements a **line-following robot** using **Hardware-in-the-Loop (HiL) simulation** with **Webots** and an **ESP32 microcontroller running MicroPython**. It was developed for the final graded assignment of the *Robotics and Python* course, Lab 7.

## Overview

The robot receives sensor data from Webots over serial, processes it in real-time using code running on the ESP32, and sends motor speed commands back to Webots. The setup mimics real-world embedded control and demonstrates line-following behavior using simulated ground sensors.

## Repository Structure

```
line-following-hil-robot/
│
├── micropython/
│ └── main.py
│
├── webots_controller/
│ └── line_following_with_HIL.py
│
├── README.md
└── .gitignore
```

## Dependencies

### ESP32 Setup
- **Board**: ESP32 WROOM-32 DevKit
- **Firmware**: MicroPython v1.25.0
- **IDE**: Thonny or any other IDE that supports micropython, for uploading `main.py`
- **Connection**: USB (UART1, 115200 baud)

### Webots Setup
- **Simulator**: Webots R2023a
- **Python Version**: Python 3.10.5 (64-bit)
- **Webots World**: Lab 7 world with `e-puck` robot

## How to Run

### 1. Flash and Upload to ESP32
1. Flash MicroPython firmware if not already installed.
2. Upload `main.py` to the ESP32.
3. Open the serial monitor (e.g., Thonny).
4. Press the left button on the ESP32 to begin.

### 2. Run Webots Simulation
1. Open Webots and load the Lab 7 world.
2. Set the robot controller to `line_following_with_HIL`.
3. Change the serial interface to the one you are using.
4. Start the simulation.
5. The robot will begin receiving sensor data and send motor commands over USB.



---

