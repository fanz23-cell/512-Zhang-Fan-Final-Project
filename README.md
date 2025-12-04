# 512-Zhang-Fan-Final-Project
# Body-Motion OLED Game with Dual IMUs

This project is a tiny “body-controlled” game that runs on a microcontroller with an OLED display.  
Two IMU sensors (one on the hand, one on the thigh) detect different motion patterns and control a small character on a 2D grid. The character can move, dodge, and turn based on how the user moves their body.

## Overview

- The **right-hand IMU** detects arm gestures like forward swing, left/right swing, and circular motions.
- The **thigh IMU** detects leg motions like left/right kicks and running.
- The microcontroller classifies each motion into actions (e.g., “前挥手”, “左转圈”, “小跑”) using template matching.
- A **16×3 grid** is drawn on the 128×64 OLED display, showing:
  - The player as a small dot.
  - Simple obstacles and targets.
  - Text labels of the currently detected hand and leg actions.

The goal is to turn abstract IMU signals into an intuitive, playful, and explainable control interface.

## Hardware

- **Microcontroller**: Xiao ESP32-C3 (or similar ESP32 board running CircuitPython)
- **IMU sensors**:  
  - 1 × ADXL345 (right hand, I2C address `0x53`)  
  - 1 × ADXL345 (thigh, I2C address `0x1D`)
- **Display**:  
  - SSD1306 128×64 OLED, I2C, device address `0x3C`
- **Power**:
  - USB 5 V input
  - On-board 3.3 V regulator powers ESP32, IMUs, and OLED

All devices share the same I2C bus (SDA/SCL) and common 3.3 V / GND.

## Folder Structure

```text
.
├─ README.md
├─ src/
│  └─ main_game.py          # Final game script (CircuitPython)
└─ Documentation/
   ├─ BodyMotion_IMU_OLED.kicad_sch       # KiCad circuit schematic
   └─ BodyMotion_System_Block_Diagram.png # System block diagram
