# Prototype 1

## 1. Overview
We are building our first prototype out of lego parts to make changing the chassis easier. This prototype is primarily for the purpose of testing of components and systems as well as finding an optimized chassis design before building the final robot. It uses a single rear-wheel drive motor and a steering mechanism controlled by a servo motor. It has a parallel steering geometry.

## 2. Materials
The robot contains the following components:-
- 1x ESP32 Devkit
- 1x LM2596 Buck Convertor
- 1x 11.1v Li-ion Battery
- 1x BNO055 IMU Sensor
- 2x TF-Luna LiDAR Sensors
- 1x L293D Motor Driver IC
- 1x DC TT Motor (Yellow)
- 1x MG90 Servo Motor


## 3. Mechanical Design
- Real-wheel drive robot
- Parallel steering geometry with front wheels steered by an MG90 servo motor
- Rear wheels are powered by a DC motor
- LiDAR sensors on left and right side


## 4. Electronics, Wiring and Power
A perfboard is used on which the circuit connections have been made. An 11.1v Li-ion battery powers the robot. This battery is connected to a buck converter which converts it to 5v so that the ESP32 can be powered. The same 5v is given to the LiDAR sensors and servo motor. The ESP32 further powers the IMU with 3.3v.

## 5. Communication Overview
The two LiDAR sensors are connected to the ESP32 via UART connections. The BNO055 is connected to the ESP32 via an I2C conncetion. The ESP32 sends data to the L293D motor driver IC by digital outputs for direction, and a PWM output for speed control.