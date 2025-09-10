Pico-W Autonomous Data Acquisition Rover

A simulation of the rover in action on the Wokwi platform.

Project Overview

This project is a sophisticated, autonomous data acquisition rover built around the Raspberry Pi Pico W. It is designed to navigate environments that are difficult or unsafe for humans, collecting and logging crucial environmental data. The entire project is developed and simulated on the Wokwi online platform, allowing for rapid prototyping and testing without any physical hardware.

The rover's core functionality includes autonomous navigation using an ultrasonic sensor for obstacle avoidance and an MPU6050 gyroscope for precise, calibrated turns. It is driven by two NEMA 17 stepper motors controlled by A4988 drivers, ensuring accurate movement.

Core Features

Autonomous Navigation: The rover moves forward until an obstacle is detected by the HC-SR04 ultrasonic sensor.

Precision Turning: Upon detecting an obstacle, the rover executes an accurate 90-degree turn using real-time data from the MPU6050 gyroscope.

Accurate Motor Control: Dual A4988 drivers provide precise control over two NEMA 17 stepper motors.

Real-Time Data Acquisition: The rover collects data from multiple sensors every 3 seconds:

Temperature: A DS18B20 digital temperature sensor provides precise readings.

Moisture: A potentiometer is used to simulate an analog soil moisture sensor.

Distance: The ultrasonic sensor continuously measures the distance to the nearest obstacle.

Status Logging: The rover's current status (e.g., "Moving Forward," "Turning") and all sensor data are logged to the serial monitor for real-time monitoring.

Hardware Components

This project is entirely simulated in Wokwi and uses the following components:

Microcontroller: Raspberry Pi Pico W

Motor Drivers: A4988 Stepper Motor Driver (x2)

Motors: NEMA 17 Stepper Motor (x2)

Gyroscope: MPU6050 6-Axis Gyroscope/Accelerometer

Temperature Sensor: DS18B20 Digital Temperature Sensor

Distance Sensor: HC-SR04 Ultrasonic Sensor

Moisture Sensor (Simulated): Potentiometer

Indicator: White LED (Flashlight)

Resistors:

4.7kΩ (Pull-up for DS18B20)

220Ω (Current-limiting for LED)

Prototyping: Mini Breadboard and Wires

Software and Setup

The project is contained within two main files, designed for the Wokwi simulation environment.

main.c: The firmware for the Raspberry Pi Pico W, written in C/C++ using the Pico SDK. It contains all the logic for motor control, sensor reading, and autonomous navigation.

diagram.json: The Wokwi diagram file that defines the complete circuit, including all components and their connections.

How to Run the Simulation

Open the project in the Wokwi Simulator.

Copy the contents of pico_data_rover_final.cpp into the main.c tab.

Copy the contents of diagram.json into the diagram.json tab.

Click the "Start Simulation" button.

Observe the rover's movement in the simulation window and view the data output in the Serial Monitor.

Future Development Ideas

Implement Wi-Fi: Utilize the Pico W's onboard Wi-Fi to send sensor data to a remote server or a real-time web dashboard.

Expand Sensor Array: Add more sensors, such as a gas sensor (MQ series), a light sensor (LDR), or a barometer.

Physical Prototype: Build a physical version of the rover using a 3D-printed chassis.

Battery Power: Incorporate a battery and power management system for true untethered operation.
