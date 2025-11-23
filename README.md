# Line-following Robot for RoboticsChampionship (2025)

This project is a line-following robot, developed using an ESP32 single-board microcontroller, designed to take part in RoboticsChampionship 2025 in Oradea. The robot uses PID (Proportional-Integral-Derivative) control to adjust its motors based on feedback from a line sensor array.

## Features

- **PID Control**: The robot uses PID to adjust motor speeds for smooth and precise line-following.
- **EEPROM Usage**: The robot uses the microcontroller's EEPROM to save and load the sensor calibration.

## Components

### Hardware
1. **[ESP32 Pico C3]([https://www.optimusdigital.ro/en/esp32-boards/12933-plusivo-esp32-and-ble-compatible-wireless-development-board.html?search_query=esp32&results=38](https://ardushop.ro/ro/electronica/537-groundstudio-carbon-v3-6427854000392.html))** - Microcontroller used to process sensor data and control the motors.
2. **[DFRobot DRI0040]([https://www.pololu.com/product/2135](https://wiki.dfrobot.com/HR8833_Dual_DC_Motor_Driver__SKU_DIR0040_))** - Dual motor driver for controlling two motors independently.
3. **[Pololu QTR-8RC Reflectance Sensor Array]([https://www.pololu.com/product/4353](https://www.pololu.com/product/961))** - High-precision sensor array for line detection.
4. **[Pololu DC Motors](https://www.pololu.com/product/3062)** - Pololu 10:1 Micro Metal Gearmotor HP 6V.
5. **[Chassis and Mounts]** - 3D-printed parts (see 3D Model section) for housing the sensors, motors, and ESP32.
6. **[Battery](https://hpi-racing.ro/li-po-2s-74v/acumulator-lipo-gens-ace-g-tech-soaring-450mah-74v-30c-2s1p-cu-jst-syp.html)** - Lipo Gens Ace Acumulator - G-Tech Soaring - 450mAh - 7.4V - 30C - 2S1P with JST-SYP
7. **Miscellaneous** - Jumper wires, connectors, screws, PCB and other hardware for assembly.

### Used Software Applications
- **ArduinoIDE**
- **Autodesk Inventor Proffesional 2024**
- **EasyEDA**

### Software Libraries
- **MotorShield** - Custom library for motor control.
- **BluetoothSerial** - For controlling the robot and transmiting data for debugging and tuning the PID.
- **EEPROM** - For accesing the microcontroller's EEPROM.
- **QTRSensors** - For reflectance sensor array control.

## Circuit Diagram

Connect the components as follows:
- **DRI0040 Motor Driver**: Connect M1DIR, M1PWM, M2DIR, M2PWM pins to custom GPIO pins 32, 33, 26, and 25 respectively.
- **QTR-8RC Sensor Array**: Connect the odd sensor pins as the Circuit Diagram shows.
- **ESP32 Power and Ground**: Ensure a stable power source is connected to the ESP32 and motors.

## 3D Model

The chassis and sensor mounts are designed to be 3D-printed for robustness and optimal sensor positioning. The STEP files for 3D printing can be found in the `3d-model` file in this repository.

## Tuning the PID Control
- **Base Speed**: Sets the default motor speed.
- **Kp, Ki, Kd**: PID control values to balance proportional, integral, and derivative responses.
- **Edge Cases**: Cases for tight corners.


