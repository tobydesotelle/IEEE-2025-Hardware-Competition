This directory contains the firmware and related code for the OpenCR board, which interfaces with sensors and actuators on the autonomous robot designed for the IEEE SoutheastCon 2025 Hardware Competition.

The OpenCR board is responsible for low-level hardware control, including:

Communicating with Dynamixel actuators (e.g., XL430 series)

Reading sensors (e.g., encoders, IMUs)

Publishing relevant data to ROS topics via serial

Receiving motion commands (e.g., wheel velocities or joint angles) from ROS
