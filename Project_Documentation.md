# 🚗 Autonomous Lane Detection & AI Vision System
**Cre8tonics Project Documentation**

---

## 📋 1. Project Overview
This project is an advanced, edge-computing Autonomous Vehicle system. It features real-time **Lane Detection** running directly on a microcontroller, paired with a stunning **Tactical Web Dashboard**, and integrated with **Cloud-based Vision AI (Groq)** for obstacle analysis. 

Unlike traditional setups that require a heavy Raspberry Pi or laptop to process video, this system performs mathematical image processing (Canny Edge & Hough Transform) *natively* on an ESP32-CAM.

---

## 🛠️ 2. Hardware Architecture

### **1. ESP32-CAM (AI-Thinker)**
* **Role:** The "Brain" and "Eyes" of the system.
* **Function:** 
  * Captures live video feed using the OV2640 camera module.
  * Processes frames pixel-by-pixel to detect road lanes.
  * Hosts the local WiFi Web Server and the HTML/JS Tactical Dashboard.
* **Camera Config:** Runs at `FRAMESIZE_QVGA` (320x240) in `PIXFORMAT_GRAYSCALE` to optimize memory and processing speed.

### **2. Raspberry Pi Pico (RP2040)**
* **Role:** The "Muscles" of the system.
* **Function:** 
  * Reads data from Ultrasonic Sensors (HC-SR04).
  * Executes object avoidance logic (stopping/turning when an obstacle is < 15cm away).
  * Controls the L298N Motor Driver via PWM signals to drive the wheels.

### **3. Actuators & Sensors**
* **Motors:** DC Gear motors driven by an L298N.
* **Ultrasonic Sensor (HC-SR04):** Measures distance to physical obstacles.
* **Power Supply:** 18650 Li-ion battery pack with a buck converter to provide clean 5V to the ESP32 and Pico.

---

## 💻 3. Software Architecture

### **1. Edge Image Processing (C++)**
The ESP32-CAM does not just stream video; it processes it. We implemented a custom vision pipeline in `main.cpp`:
1. **Gaussian Blur:** Smooths the image to remove camera noise.
2. **Sobel Edge Detection:** Finds sharp gradients (edges) in the image.
3. **Thresholding:** Converts the edge map into a pure binary (Black & White) image.
4. **Hough Line Transform:** Mathematically searches the binary image to find straight continuous lines (the road lanes).
5. **Steering Calculation:** Calculates the vanishing point of the left and right lanes to determine if the car needs to steer Left, Right, or go Straight.

### **2. Tactical Web Dashboard (HTML/CSS/JS)**
* Served directly from the ESP32-CAM's memory.
* Features a highly responsive, futuristic "Cyberpunk / Neon Cyan" aesthetic.
* **Live Overlay:** The ESP32 sends the calculated lane coordinates to the dashboard, and the browser draws neon red/green lines *over* the live video stream using an HTML5 `<canvas>`.

### **3. AI Vision Assistant (Groq API Integration)**
* **The Problem:** The ESP32-CAM does not have enough RAM to establish an SSL/HTTPS connection to cloud AI servers.
* **The Solution:** We implemented a brilliant workaround. When you click the **[ ASK AI ]** button, the *browser* grabs a high-resolution snapshot from the video canvas, encodes it in Base64, and sends it directly to the **Groq AI Vision API** (using models like Qwen or LLaMA-Vision). 
* **Result:** The car can literally "see" an obstacle, send it to the cloud, and the AI replies with a human-readable sentence (e.g., *"There is a red stop sign blocking the path."*).

---

## 🐛 4. Challenges Solved (Q&A)

**Q: Why was the ESP32-CAM crashing with a `malloc failed` or `LoadStoreError` initially?**
**A:** The ESP32 has limited contiguous internal memory (DRAM). When we tried to process uncompressed RGB Color images, it required 153KB of contiguous RAM, which the ESP32 couldn't allocate. We solved this by switching the edge-detection pipeline to `PIXFORMAT_GRAYSCALE`, which slashed the memory requirement in half (76KB) and completely eliminated the crashes.

**Q: How does the system handle AI requests without crashing the ESP32?**
**A:** Microcontrollers struggle with heavy HTTPS encryption. Instead of forcing the ESP32 to talk to the Groq API, we offloaded that task to the Client's Web Browser. The ESP32 simply serves the webpage; the JavaScript inside the webpage makes the heavy API calls.

**Q: Why did we split the workload between an ESP32-CAM and a Pico?**
**A:** The ESP32-CAM is notoriously short on available GPIO pins (most are used by the camera and SD card). By letting the ESP32 handle Vision/WiFi and letting the Pico handle Motors/Sensors, we created a highly stable, modular dual-core system.

---

## 🚀 5. How to Run the Project

1. **Upload ESP32 Code:** 
   * Open the `esp32_cam_lane` folder in PlatformIO.
   * Ensure `platformio.ini` is set to `board_build.partitions = huge_app.csv`.
   * Run `pio run -t upload`.
2. **Upload Pico Code:** 
   * Use `mpremote cp pico_object_avoidance.py :main.py`.
   * Reset the Pico.
3. **Operate:**
   * Open the Serial Monitor to find the ESP32's IP Address (e.g., `http://192.168.x.x`).
   * Connect your phone or laptop to the same WiFi network and open the IP.
   * Watch the autonomous lane detection run live, and click **[ ASK AI ]** to analyze the environment!
