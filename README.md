# Image Processing Based Autonomous Fire Detection & Extinguishing Robot

A low-cost, distributed autonomous fire-fighting robot developed as a final project for EEE 318 at BUET. 

This system separates the "Brain" (a centralized laptop running deep learning models) from the "Muscle" (a lightweight ESP32 mobile robot), allowing for zero-lag processing and highly precise physical execution.

## System Architecture

* **The Command Center (Node 1):** An overhead camera feeds into a laptop running a custom-trained **YOLOv8** model to detect the exact base coordinates of a fire. It simultaneously uses **OpenCV and ArUco markers** to track the robot's physical location and heading.
* **The Mobile Unit (Node 2):** An **ESP32** microcontroller receives discrete steering commands ('F', 'L', 'R', 'S') over a low-latency UDP Wi-Fi connection. It executes closed-loop burst movements using hardware interrupts and optical wheel encoders to prevent slip-induced overshoot.

##  Hardware Components
* **Microcontroller:** ESP32 DevKit V1
* **Motors:** 4x TT Gear Motors (4WD) driven by 2x L298N Dual H-Bridges
* **Sensors:** 2x LM393 Optical Wheel Encoders, 2x LM393 IR Flame Sensors
* **Extinguisher:** 5V Submersible Water Pump via Relay Module + MG996R Servo for aiming
* **Power:** 11.1V 3S Li-Po Battery regulated via LM2596 Buck Converter

##  Software Stack
* **Python:** YOLOv8 (Ultralytics), OpenCV (ArUco Detection), UDP Sockets
* **C++ (Arduino IDE):** ESP32 WiFi UDP handling, FreeRTOS background tasks, Hardware Interrupts

##  Key Features
* **Dynamic Scale Calibration:** Automatically recalculates pixels-per-centimeter based on ArUco marker size to eliminate camera height distortion.
* **Fuel-Source Targeting:** Tracks the bottom-center of the YOLO bounding box to aim at the fuel source, ignoring flickering flames.
* **Visual Servoing:** Translates geometric distance and heading errors into discrete encoder "slots" for precise stop-and-go navigation.
* **Smart UDP Flush:** Ensures critical stop commands (`S`) are never lost during burst movements.

## 👥 The Team (Group 04.B1)
* Shaishab Saha
* Mrinmoy Das Pranto
* Md. Shafiul Alam
* S M Abir Hasan
* Abrar Zawad

Department of Electrical and Electronic Engineering, BUET
