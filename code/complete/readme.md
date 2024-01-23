# Complete (adjustable controll with display)

This readme provides detailed information about the PID Motor Control code, guiding users through its setup, components, and functionality.

## Table of Contents

1. [Introduction](#introduction)
2. [Components](#components)
3. [Setup](#setup)
4. [LCD Display](#lcd-display)
5. [Ultrasonic Sensor](#ultrasonic-sensor)
6. [Potentiometers](#potentiometers)
7. [PID Calculation](#pid-calculation)
8. [PWM Output](#pwm-output)
9. [Adjustable Parameters](#adjustable-parameters)
10. [Usage](#usage)
11. [Troubleshooting](#troubleshooting)

## Introduction

The PID Motor Control code is designed for Arduino-based projects, implementing a Proportional-Integral-Derivative (PID) controller to regulate the movement of a motor. The system utilizes an ultrasonic sensor to measure distance and adjust the motor's speed accordingly.

## Components

- Arduino Board
- Ultrasonic Sensor (HC-SR04)
- LCD Display (I2C)
- Potentiometers (for Setpoint, Kp, Ki, Kd)
- Motor
- Wires and Breadboard

## Setup

1. Connect the components according to the provided pin configurations in the code.
2. Ensure the correct address for the I2C LCD is set (default: 0x27).
3. Upload the code to your Arduino board.

## LCD Display

The LCD provides information about setpoint, actual distance, PWM value, and PID value. The initial display shows a welcome message.

## Ultrasonic Sensor

The ultrasonic sensor measures distance (in cm) and influences the PID controller's behavior based on the setpoint.

## Potentiometers

Potentiometers adjust the setpoint, and PID parameters (Kp, Ki, Kd). Changes in potentiometer values are reflected on the LCD.

## PID Calculation

The PID controller calculates the error, integral, and derivative terms to determine the appropriate PWM output for motor control.

## PWM Output

The PWM output is adjusted based on the PID calculation, controlling the motor's speed. The PWM range is constrained between 143 and 154.

## Adjustable Parameters

- **Setpoint:** Adjusted using a potentiometer.
- **Kp, Ki, Kd:** Tuned using respective potentiometers.

## Usage

1. Power up the system.
2. Observe LCD for distance, setpoint, and PID values.
3. Adjust potentiometers for real-time changes.
4. Troubleshoot, if needed (refer to Troubleshooting section).

## Troubleshooting

- **No Display:** Check connections and LCD address.
- **Incorrect Sensor Readings:** Inspect ultrasonic sensor and wiring.
- **Motor Not Responding:** Verify PWM values and motor connections.

## License

This project is licensed under the [MIT License](../../LICENSE).
