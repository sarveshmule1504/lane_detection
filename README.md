# Lane Detection Smart Vehicle

This project is a simple autonomous car system that can:
- detect road lanes in real time,
- avoid obstacles using an ultrasonic sensor,
- control a car using motors,
- send live camera feed to a dashboard,
- and use AI to identify objects in front of the car.

The project combines hardware and software to make a small smart vehicle that can move on a road and react to its surroundings.

## What this project does

The car uses an ESP32-CAM to capture video from the front camera. It processes the image to detect lane lines and decide whether the robot should move left, right, or straight.

At the same time:
- a Raspberry Pi Pico reads distance information from an ultrasonic sensor,
- the motors are controlled to move the car,
- the system can stop or turn if an obstacle is too close,
- and an AI model can describe what is in front of the car.

## Main features

- Real-time lane detection
- Object detection and avoidance
- Motor control using L298N motor driver
- Camera stream dashboard
- AI-based visual analysis using Groq API
- Small embedded system design suitable for a smart vehicle project

## Hardware used

- ESP32-CAM
- Raspberry Pi Pico (RP2040)
- L298N motor driver
- DC motors
- Ultrasonic sensor (HC-SR04)
- Battery pack and power system
- Camera module

## Project structure

- `esp32_cam_lane/` - ESP32 code for camera, lane detection, and web dashboard
- `ai_dashcam.py` - Python script to connect to the ESP32 camera stream and send frames to AI
- `raspi_controller.py` - Python script for Pi or controller-related logic
- `pico_object_avoidance.py` - Pico code for obstacle avoidance
- `motor_test.py` - Simple script to test motor movement
- `blink.py` - Basic LED test script
- `Project_Documentation.md` - More detailed explanation of the project

## How the system works

1. The camera captures the road image.
2. The ESP32 processes the frame and looks for lane lines.
3. Based on the lane position, the system decides the steering direction.
4. If an obstacle is too close, the car stops or changes direction.
5. The live feed can be viewed through a local web dashboard.
6. The AI assistant can analyze the image and tell what is in front of the vehicle.

## Requirements

### Software

- PlatformIO for ESP32 development
- Python 3
- OpenCV
- Requests library
- A Groq API key for AI image analysis

### Hardware setup

- Connect the ESP32-CAM to the car chassis
- Connect the Pico to the motor driver and sensors
- Power the motors and controller properly
- Make sure the ESP32 and Pico are connected to the same network if using the dashboard

## Setup steps

### 1. ESP32 project

Open the `esp32_cam_lane` folder in PlatformIO and upload the code to the ESP32.

Use the project configuration from `platformio.ini` and make sure the board settings match your ESP32-CAM module.

### 2. Raspberry Pi Pico project

Upload the Pico script using a tool such as `mpremote`.

Example:

```bash
mpremote cp pico_object_avoidance.py :main.py
```

### 3. Python scripts

Install the needed Python packages:

```bash
pip install opencv-python requests
```

Then update the API key in `ai_dashcam.py` before running it.

## Run the project

### Lane detection / dashboard

- Power the ESP32-CAM
- Connect to the Wi-Fi network it creates or joins
- Open the IP address shown on the serial monitor
- View the live road feed and lane detection results

### AI image analysis

Run:

```bash
python ai_dashcam.py
```

Then press the space key to capture the current frame and send it to the AI model.

## Important notes

- Put your real Groq API key in `ai_dashcam.py`.
- Make sure the camera URL and Wi-Fi settings are correct.
- The car should be tested carefully in a safe environment.
- This project is educational and can be used for robotics or embedded systems learning.

## Project goal

This project is a good example of how embedded systems, computer vision, and AI can be combined to build a smart self-driving vehicle prototype.

## Future improvements

- Add better steering logic
- Improve lane detection under poor lighting
- Add more obstacle detection rules
- Add a mobile app or better dashboard
- Add camera calibration and threshold tuning

## License

This project is provided for learning and experimentation. Please check the repository for any specific licensing information if needed.

## Summary

This project teaches how a robot car can see the road, understand its environment, and react without much human input. It is a practical way to learn about computer vision, microcontrollers, motor control, and AI integration.
