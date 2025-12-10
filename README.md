# **IMU-Driven Action Game on ESP32-C3 + OLED**
*A full-body motion-controlled pixel game built with three ADXL345 sensors, real-time motion classification, and a scrolling 2× world map.*

---

## **1. Project Overview**

This project is a **gesture-controlled action game** running entirely on a **Seeed Studio XIAO ESP32-C3**, an **SSD1306 128×64 OLED display**, and **three ADXL345 accelerometers** mounted on the **right hand, right leg, and left leg**.  
The player physically performs gestures—like *hand swings, leg swings, turning, running, attacking*—and the on-screen 4×4 avatar moves through a **large 2× world map**, collecting hollow balls before the level timer expires.

Game logic is implemented in the main program.  
Level geometry and spawn points are stored in a separate `level_maps.py` file.

---

## **2. How the Game Works**

### **2.1 Motion → Action Classification**

Each ADXL345 sensor provides 3-axis acceleration. The system:

1. Applies **low-pass filtering** to stabilize noisy IMU data  
2. Converts acceleration to features:  
   - pitchX / pitchY / pitchZ  
   - magnitude  
3. Classifies gestures using **pre-trained templates** (mean/std)  
4. Applies additional logic:  
   - stillness detection  
   - trend detection (continuous pitch change for turning)  
   - sliding-window peak detection for running  
   - leg-swing cooldown  
5. Converts recognized gestures into game actions:

**Hand IMU controls direction & attack:**
- Forward swing → move forward  
- Left swing → strafe left  
- Right swing → strafe right  
- Circle gestures → curved rotation  
- Attack gesture → extend arrow & pop balls  

**Leg IMUs control turning & running:**
- Left leg swing → rotate 90° left  
- Right leg swing → rotate 90° right  
- Running → continuous forward movement  

---

## **3. Game Flow**

### **3.1 Difficulty Selection**

Before gameplay, the **rotary encoder** selects:
- Easy  
- Medium  
- Hard  

The encoder push-button confirms the choice.  
Each difficulty sets a different per-level countdown timer.

### **3.2 Level System**

There are **10 levels**, each containing:
- A 2× scaled world (256×128 virtual space)  
- Unique wall structures  
- A defined spawn location  
- Ten hollow collectible balls  

If all balls are collected →  
**“Level X complete”** is shown and the next level loads.

If time runs out →  
**“fail”** is displayed.

After level 10 →  
**“All levels complete!”** is shown.

### **3.3 Rendering Pipeline**

Every frame:
1. Compute camera viewport relative to the player  
2. Draw walls clipped to the viewport  
3. Draw hollow balls  
4. Draw the 4×4 player  
5. Draw direction arrow  
6. Draw a mini-map (32×16)  
7. Draw bottom-left HUD:  
