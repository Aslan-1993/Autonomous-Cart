# 🤖 Autonomous Cart IR-Guided with Obstacle Avoidance

"""It is recommended to read the PDF for full technical details: Autonomous_Cart.pdf"""

## 🔧 Project Overview

An intelligent autonomous cart built with an **Arduino Mega**, capable of:

- 📡 Locating and following a modulated IR signal
- 🚧 Avoiding static obstacles using ultrasonic sensors
- 🔄 Real-time path correction based on sensor feedback
- 📲 Wireless activation via Bluetooth (HC-06)

> Designed for indoor flat-terrain navigation and tested under lab conditions.

---

## 🧠 System Features

| Feature                     | Description                                         |
|----------------------------|-----------------------------------------------------|
| IR Tracking                | Locates the 38kHz IR signal modulated at 150/300Hz |
| Obstacle Detection         | Avoids obstacles using ultrasonic HC-SR04 sensors  |
| Directional Decision Logic | Calculates best route using 3 IR sensors           |
| Bluetooth Control          | Enables remote ON/OFF from mobile device           |
| Autonomous Navigation      | No human intervention needed after activation      |

---

## 🧰 Tools & Components

| Component             | Description                                |
|----------------------|--------------------------------------------|
| Arduino Mega 2560     | Main microcontroller for cart             |
| Arduino Nano          | IR transmission module                    |
| TSOP4838              | IR receiver sensors (front, left, right)  |
| IR333A LED + BC547    | 38kHz IR signal emitter circuit           |
| HC-SR04               | Ultrasonic distance sensors (x4)          |
| L293D Motor Shield    | Dual H-Bridge driver for 4 DC motors      |
| TT130 DC Motors (x4)  | Power the cart movement                   |
| HC-06 Bluetooth       | For manual wireless control (ON/OFF)      |
| 18650 Li-Ion Batteries| Power source (11.1V, 2.8A)                |

---

## 📊 Functional Highlights

| Module         | Capabilities                                                             |
|----------------|--------------------------------------------------------------------------|
| IR Receiver    | Tracks signal direction to guide cart                                    |
| Obstacle Unit  | Detects objects <35cm and alters movement path                           |
| Drive System   | Supports forward, reverse, and pivot turning using motor control matrix  |
| Bluetooth      | Accepts basic start/stop command remotely                                |
| Transmission   | IR pulse modulated using Sony protocol @ 38kHz (150Hz / 300Hz toggle)    |

---

## ⚙️ Setup Instructions

### 1. Install Arduino IDE  
[Download Arduino IDE](https://www.arduino.cc/en/software)

### 2. Required Libraries
Make sure you have these installed:
- `AFMotor` (Adafruit Motor Shield library)
- `SoftwareSerial` (built-in)
- `IRremote` (for the IR transmitter)
- `NewPing` (optional for HC-SR04 handling)
