OpenCR Firmware – IEEE 2025 Hardware Competition
This directory contains the firmware and related code for the OpenCR board, which interfaces with sensors and actuators on the autonomous robot designed for the IEEE SoutheastCon 2025 Hardware Competition.

The OpenCR board is responsible for low-level hardware control, including:

Communicating with Dynamixel actuators (e.g., XL430 series)

Reading sensors (e.g., encoders, IMUs, TCRT5000 line sensors)

Publishing relevant data to ROS topics via serial

Receiving motion commands (e.g., wheel velocities or joint angles) from ROS

📦 Contents
makefile
Copy
Edit
opencr/
├── include/                 # Header files
├── src/                     # Main source files
├── lib/                     # Dynamixel SDK and other hardware libraries
├── CMakeLists.txt           # Build configuration for Arduino framework
└── README.md                # This file
🔧 Hardware Setup
Board: OpenCR 1.0 or 2.0

Actuators: Dynamixel XL430-W250-T

Sensors:

TCRT5000 Reflective Sensors (line detection)

Adafruit BNO055 (9-DoF IMU, I2C)

Optional encoders or float switches

Communication: USB or serial link to Raspberry Pi or Jetson host running ROS 2

🚀 Features
Velocity Control: Accepts wheel velocity commands via serial and drives motors accordingly

Sensor Feedback: Reads IMU, line sensors, or encoders and transmits data to host

Homing Mode: Dynamixel actuators can perform a current/load-based homing on startup

ROS Interface: Custom serial protocol for publishing/subscribing to ROS topics through UART

📥 Installation
Install Arduino IDE

https://www.arduino.cc/en/software

Install OpenCR Board Support

Follow instructions here: ROBOTIS OpenCR Setup

Clone the repo and open the project:

bash
Copy
Edit
git clone https://github.com/tobydesotelle/IEEE-2025-Hardware-Competition.git
cd IEEE-2025-Hardware-Competition/opencr
Upload firmware to OpenCR

Open the main .ino or .cpp file in Arduino IDE

Select Tools > Board > OpenCR

Choose the correct port

Upload

🖧 Serial Protocol
The OpenCR board communicates with the main robot computer using a simple serial protocol. For example:

VEL:x,y,z\n – Set wheel/local velocities

PING\n – Heartbeat or system check

SENSOR?\n – Request sensor data

Customize the protocol in src/main.cpp or equivalent.

🧪 Testing
You can test the firmware standalone by opening the Arduino Serial Monitor or using minicom/screen. Make sure the baud rate matches (typically 57600 or 115200).

For integration with ROS, use a serial node (e.g., ros2_serial_bridge or a custom Python script) to parse and relay data.

🧠 Notes
Homing logic is implemented based on Dynamixel present current. If the motor stalls for a threshold period, it switches to position mode.

IMU data is published in quaternion format; raw acceleration and gyro values are also available.

The code is modular to allow easy swapping of sensor/actuator hardware.

📄 License
This project is licensed under the MIT License. See the LICENSE file for details.

🙌 Credits
Developed by the UCF Robotics Team for IEEE SoutheastCon 2025. Powered by OpenCR, Dynamixel, and ROS 2.

Let me know if you’d like this tailored to OpenCR 1.0 vs 2.0, or if you want integration examples with your src_ws or ROS packages.
