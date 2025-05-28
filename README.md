# Line-Following HiL Robot

This project implements a **line-following robot** using **Hardware-in-the-Loop (HiL) simulation** with **Webots** and an **ESP32 microcontroller running MicroPython**. It was developed as the final graded assignment (Lab 7) for the *Robotics and Python* course at the Hanze.

> 📚 Full assignment brief: [Lab 7 – Hardware-in-the-Loop Simulation](https://felipenmartins.github.io/Robotics-Simulation-Labs/Lab7/)

## Overview

The ESP32 receives sensor data from Webots over a wireless TCP socket, processes it in real-time using MicroPython, and sends motor speed commands and its planned path back to Webots. The simulation mimics real-world embedded control and showcases line-following behavior with simulated ground sensors.

Key features:
- Real-time HiL control loop via Wi-Fi
- Path planning using ESP32
- Dynamic re-planning and path visualization via Streamlit

## Repository Structure

```
line-following-hil-robot/
│   README.md
├───Assignment 7/
│   ├───micropython/
│   │   ├── communicator.py
│   │   ├── linetracer.py
│   │   ├── main.py
│   │   ├── odometer.py
│   │   ├── pid.py
│   │   ├── robot.py
│   │   └── tomtom.py
│   ├───webots_controller/
│   │   ├── communicator.py
│   │   ├── line_following_with_HIL.py
│   │   └── robot.py
│   └───website/
│       ├── app.py
│       ├── map.png
│       └── robot.png
├───Electrical Drawings/
│   └── S-005_SensorArray_Schema.qet
└───Simulation/
    ├───controllers/
    │   └── main/
    │       ├── linetracer.py
    │       ├── main.py
    │       ├── pid.py
    │       ├── robot.py
    │       └── tomtom.py
    ├───protos/
    │   ├── E-puckGroundSensors.proto
    │   └── icons/
    │       ├── E-puck.png
    │       └── E-puckGroundSensors.png
    └───worlds/
        ├── RaFLite.wbt
        ├── RaFLite.wbproj
        ├── RaFLite_track.png
        └── textures/
```

## Dependencies

### ESP32 Setup
- **Board**: ESP32 WROOM-32 DevKit
- **Firmware**: MicroPython v1.25.0
- **IDE**: Thonny (or any MicroPython-compatible IDE)

### Webots Setup
- **Simulator**: Webots R2023a
- **Python**: Version 3.10.5 (64-bit)
- **Robot Model**: e-puck with ground sensors

### Python Packages
Install with `pip install -r requirements.txt`, or manually:
```
streamlit
Pillow
streamlit-autorefresh
```

## How to Run

### 1. Flash and Upload to ESP32
1. Flash the ESP32 with MicroPython firmware.
2. Upload all files from `Assignment 7/micropython/` to the ESP32.

### 2. (Optional) Start the Streamlit Visualization
1. Navigate to `Assignment 7/website/`.
2. Run: `streamlit run app.py`
3. A browser window will open. Wait for the robot to find a path.

### 3. Run the Webots Simulation
1. Open Webots and load the `RaFLite.wbt` world.
2. Set the controller of the e-puck robot to `line_following_with_HIL`.
3. Connect your computer to the ESP32's Wi-Fi.
4. Start the simulation — the robot will begin receiving data and controlling its movement wirelessly.

