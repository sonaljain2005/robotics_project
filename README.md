# Robo Laser Tag - Autonomous Mobile Robot

## Overview

The goal was to design and implement a fully autonomous robot capable of navigating an enclosed arena, detecting and eliminating targets using a laser, avoiding obstacles, and defending itself from rear attacks. The robot combines a variety of sensors and actuators controlled by an Arduino MEGA, with thoughtful algorithms enabling intelligent decision-making in a dynamic environment.

---

## Hardware Components

* **Microcontroller:** Arduino MEGA (for additional I/O pins)
* **Chassis:** Standard course kit with modifications (tank treads instead of wheels)
* **Sensors:**

  * 3 Ultrasonic Sensors (2 front, 1 rear)
  * 2 IR Receivers (mounted on servos front and rear)
  * Line Tracking Module (bottom front)
* **Actuators:**

  * 2 Servos (rotating IR receivers and laser)
  * Laser Module (mounted on front servo)
* **Other:**

  * Breadboard for power/ground routing
  * Battery pack

---

## Key Features

### 🔍 Edge & Obstacle Detection

* Front-mounted ultrasonic sensors in a criss-cross pattern detect walls and obstacles.
* Rear ultrasonic sensor detects flanking enemy robots.
* When obstacles are detected within 10 cm:

  * Turn 180° if both sensors detect.
  * Turn 60° based on which side detects the obstacle.

### 🚧 Obstacle Avoidance Algorithm

* Uses turn counts to prevent getting stuck in loops.
* Stops momentarily to check for changing distance, indicating enemy presence versus a static wall.

### 🔫 Target Detection & Elimination

* A single IR sensor mounted on a front servo scans like a radar.
* Servo stops when signal detected; laser fires for 1 second.
* Target confirmation via signal status post-firing:

  * If signal gone → headshot.
  * If signal persists → misfire, restart scan.

### 🔄 Defensive Mechanism

* Rear ultrasonic sensor checks for close opponents (within 25 cm).
* If detected and closing in, robot performs 180° turn to face attacker.

### 🧠 Target Scanning Logic

* Front and rear IR sensors handle 360° detection using split coverage.
* Scanning continues even while robot is moving to avoid being stationary.

### 📍 Line Tracking Fallback

* After 20 seconds of no target detection, the robot switches to line-following mode.
* Simple algorithm keeps dark line between sensors.

---

## Design and Construction Notes

* Front sensors were reinforced using rubber bands and double-sided tape for durability.
* The structure balances component weight to avoid forward sagging.
* Experimentation with multiple IR sensors was dropped due to mechanical instability.

---


## Challenges Faced

* Difficulty with single IR receiver accuracy at certain angles.
* Multiple receivers caused chassis imbalance and were ultimately removed.
* Ensuring reliable scanning and shooting while the robot is in motion.

---
