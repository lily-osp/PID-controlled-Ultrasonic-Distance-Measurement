# Base ver (Simple PID Motor Control)

This readme provides a comprehensive guide to the Simple PID Motor Control code. It enables users to understand the setup, functionality, and customization options of the code.

## Table of Contents

1. [Introduction](#introduction)
2. [Components](#components)
3. [Setup](#setup)
4. [Ultrasonic Sensor](#ultrasonic-sensor)
5. [Potentiometer](#potentiometer)
6. [PID Controller](#pid-controller)
7. [PWM Output](#pwm-output)
8. [Adjustable Parameters](#adjustable-parameters)
9. [Usage](#usage)
10. [Troubleshooting](#troubleshooting)

## Introduction

The Simple PID Motor Control code is designed for Arduino-based projects, implementing a basic Proportional-Integral-Derivative (PID) controller to regulate the movement of a motor. The system uses an ultrasonic sensor to measure distance and adjusts the motor's speed based on a setpoint.

## Components

- Arduino Board
- Ultrasonic Sensor (HC-SR04)
- Potentiometer
- Motor
- Wires and Breadboard

## Setup

1. Connect the components according to the provided pin configurations in the code.
2. Upload the code to your Arduino board.

## Ultrasonic Sensor

The ultrasonic sensor measures distance (in cm) and influences the PID controller's behavior based on the setpoint.

## Potentiometer

The potentiometer adjusts the setpoint, allowing users to define the desired distance for the motor to maintain.

## PID Controller

The PID controller calculates the error, integral, and derivative terms to determine the appropriate PWM output for motor control.

## PWM Output

The PWM output is adjusted based on the PID calculation, controlling the motor's speed. The PWM range is constrained between 143 and 154.

## Adjustable Parameters

- **Setpoint:** Adjusted using the potentiometer.
- **Kp, Ki, Kd:** Fixed values set in the code to define the Proportional, Integral, and Derivative gains.

## Usage

1. Power up the system.
2. Adjust the potentiometer to set the desired distance (setpoint).
3. Observe serial monitor for setpoint, actual distance, PWM value, and PID value.
4. Monitor motor behavior based on adjustments.

## Troubleshooting

- **No Serial Output:** Check connections and ensure the Arduino is connected to the computer.
- **Incorrect Sensor Readings:** Inspect ultrasonic sensor and wiring.
- **Motor Not Responding:** Verify PWM values and motor connections.

## License

This project is licensed under the [MIT License](../../LICENSE).
