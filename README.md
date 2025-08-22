# IoT Based Patient Monitoring System 🏥

This project is a **low-cost, IoT-enabled patient monitoring and safety system** built using an ESP32 and multiple sensors. It continuously monitors **heart rate, SpO2, temperature, humidity, motion, vibration, and fall detection** and uploads the data to **ThingSpeak Cloud** for near real-time visualization.

---

## 🔹 Features
- Heart rate (BPM) and SpO2 using MAX30100
- Temperature & Humidity via DHT22
- Fall detection with MPU6050 accelerometer/gyroscope
- PIR motion & vibration monitoring
- Data uploaded every 18s to ThingSpeak cloud
- Local display on 16×2 I2C LCD
- Safety actions:
  - Buzzer alert
  - Servo-based door opening
- Affordable & reliable design
---

## 🔹 IoT-Patient-Monitoring/
│
├── src/
│   └── patient_monitor.ino      <-- Arduino/ESP32 code file
│
├── images/
│   └── project-setup.jpg        <-- photo of your project
│
├── docs/
│   └── IoT_Project_Report.pdf   <-- your project report

---

## 🖥️ System Architecture
<img width="3840" height="1924" alt="Untitled diagram _ Mermaid Chart-2025-08-17-063206" src="https://github.com/user-attachments/assets/91dc46a3-a585-478d-ab5b-ae0aed136a6a" />


---

## 📷 Project Photo
![6127181940994983876](https://github.com/user-attachments/assets/1e618ccc-9e1c-46f7-93b1-ac0ee753cba5)


---

## 🛠 Hardware Components
- ESP32 microcontroller
- MAX30100 – Heart rate & SpO2 sensor
- DHT22 – Temperature & Humidity
- MPU6050 – Accelerometer/Gyroscope (Fall detection)
- PIR sensor – Motion detection
- SW-420 – Vibration sensor
- Servo Motor & Buzzer
- 16×2 LCD with I2C module

---

## 📡 Cloud Platform
- ThingSpeak for real-time monitoring
- Data uploaded every 16–20s

---

## 📑 Documentation
[IoT_Project_Report.pdf](https://github.com/user-attachments/files/21931184/IoT_Project_Report.pdf)


---

## 📦 Required Libraries
List of Arduino/ESP32 libraries to install:
- `Adafruit_MPU6050`
- `Adafruit_Sensor`
- `DHT`
- `ESP32Servo`
- `MAX30100`
- `LiquidCrystal_I2C`

---
