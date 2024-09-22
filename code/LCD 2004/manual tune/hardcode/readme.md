# Arduino PID Control with Ultrasonic Sensor and I2C LCD Display

## Overview

This Arduino project utilizes a PID (Proportional-Integral-Derivative) controller to control a motor's speed based on the distance measured by an ultrasonic sensor. The project also includes an I2C LCD display for visual feedback. The PID values and setpoint are adjusted using a potentiometer.

## Components Required

- Arduino board
- Ultrasonic sensor (HC-SR04)
- Potentiometer
- Motor
- I2C LCD display (20x4)
- Motor driver (if needed)
- Connecting wires

## Installation

### Wiring

1. Connect the ultrasonic sensor to the specified pins on the Arduino.
2. Connect the potentiometer to an analog pin on the Arduino.
3. Connect the motor to a PWM pin on the Arduino.
4. Connect the I2C LCD display to the SDA and SCL pins on the Arduino.

### Libraries

Ensure you have the required libraries installed:

- [Wire Library](https://www.arduino.cc/en/reference/wire)
- [LiquidCrystal_I2C Library](https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library)

## Code Explanation

### Functions

- `getDistance()`: Measures the distance using the ultrasonic sensor.
- `controlMotor()`: Implements PID control for motor speed based on distance.
- `updateLCD()`: Updates the I2C LCD display with feedback information.

### PID Parameters

- `Kp`, `Ki`, and `Kd`: Proportional, Integral, and Derivative gains for the PID controller.

### LCD Refresh

The LCD display is updated in a separate function (`updateLCD()`) with a customizable refresh interval to reduce flickering.

## Usage

1. Upload the code to your Arduino board.
2. Ensure all components are correctly wired.
3. Observe the feedback on the I2C LCD display and the Serial Monitor.
4. Adjust the potentiometer to change the setpoint and observe motor response.

## Customization

- Modify PID parameters (`Kp`, `Ki`, `Kd`) based on your requirements.
- Adjust the setpoint range using the potentiometer.
- Customize the LCD refresh interval in the `updateLCD()` function.

## License

This project is licensed under the [MIT License](../../LICENSE).
