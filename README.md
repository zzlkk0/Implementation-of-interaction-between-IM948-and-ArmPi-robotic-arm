# Implementation-of-interaction-between-IM948-and-ArmPi-robotic-arm
This repository contains the full implementation of a human-computer interaction system that uses a 9-axis IMU (IM948), an ESP32-C3 microcontroller, and the ArmPi robotic arm to realize real-time motion capture, filtering, visualization, and robotic control.
##  Features

- **Sensor Fusion:**  
  Kalman-filtered quaternion + acceleration data for smooth, stable motion tracking.

- **Wireless Communication:**  
  ESP32 reads IMU + ADC data and sends it via TCP to the PC for further processing.

- **Real-Time Visualization:**  
  PyQt6 interface includes:
  - 2D signal plots (filtered vs. raw)
  - 3D trajectory display
  - 3D cube rotation rendering
  - MJPEG HTTP streaming (`http://<host>:7777`)

- **Position & Angle Tracking:**  
  Dynamic compensation + trajectory reconstruction based on offset integration.

- **GUI Control:**  
  Tkinter interface allows toggling between position and angle transmission modes.

- **Robotic Arm Control:**  
  Support for:
  - Inverse kinematics-based control  
  - Direct servo angle mapping  
  via Hiwonder’s ArmPi and BusServo APIs.

##  Hardware Setup

- IM948 MEMS 9-axis IMU  
- ESP32-C3 Mini  
- Hiwonder ArmPi Robotic Arm  
- Strain Gauge + BF350 Module  
- 3.7V Li-ion battery & 3.3V regulator

##  Directory Structure

. ├── src/ │ ├── esp32_data_extract.py # TCP receiver & packet parser │ ├── filter.py # UKF filter and quaternion tools │ ├── gui_mjpeg.py # PyQt6 GUI + MJPEG server │ ├── tk_sender.py # Tkinter controller + ZMQ transmitter │ ├── run_zmq_servo.py # ZMQ angle-mapped servo control │ └── run_zmq_armik.py # ZMQ position-mapped IK control ├── config.yaml # Network/config file ├── imu_data_cache.json # Fallback IMU data cache ├── README.md # Project documentation

##  Getting Started

1. Flash the ESP32 with UART-IMU communication and TCP upload code.
2. Run `gui_mjpeg.py` to launch the live motion dashboard and HTTP stream.
3. Launch `tk_sender.py` to interactively send position or angle data.
4. Choose one of:
   - `run_zmq_servo.py` for angle control
   - `run_zmq_armik.py` for inverse kinematics control

##  Example Transmission Format

```python
[prefix, x, y, z, voltage]
# prefix: 0 = displacement mode, 1 = angle mode
# voltage: ADC value from ESP32 strain sensor
