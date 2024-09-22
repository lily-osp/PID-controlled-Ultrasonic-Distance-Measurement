# README

## 1. Introduction

This code implements a PID (Proportional-Integral-Derivative) control system that dynamically adjusts the parameters (Kp, Ki, Kd) based on real-time performance metrics. It uses an ultrasonic sensor to measure distance and controls a motor to maintain the setpoint distance. The PID controller is adaptive, using performance data to tune itself during operation.

The project also includes an LCD display for visualizing real-time PID values and system performance, along with Serial output for debugging.

## 2. Setup

To run this project, you'll need the following components:
- Ultrasonic Sensor (e.g., HC-SR04)
- DC Motor with PWM control
- Potentiometer for adjusting the setpoint
- LiquidCrystal_I2C display (20x4 or 16x2)
- Microcontroller (e.g., Arduino Uno)
- Breadboard and jumper wires

### Pin Connections:
- **Ultrasonic Sensor**: 
  - Trig to Pin 3
  - Echo to Pin 2
- **Potentiometer**: 
  - Analog Pin A0
- **Motor**: 
  - PWM Pin 9
- **LCD I2C Address**: 
  - 0x27 (adjust as needed)

## 3. Dependencies

This project requires the following Arduino libraries:
- **Wire.h**: For I2C communication.
- **LiquidCrystal_I2C.h**: To interface with the LCD over I2C.

To install the libraries:
- Open the Arduino IDE
- Go to **Sketch > Include Library > Manage Libraries**
- Search for `LiquidCrystal_I2C` and install the appropriate version.

## 4. Configuration

### PID Constants
The initial PID values are set to:
- **Kp**: 1.0
- **Ki**: 0.0
- **Kd**: 0.0

These values will be adjusted dynamically during operation based on system performance metrics. You can manually adjust these constants in the code if necessary.

### Setpoint Range
The setpoint is determined by the potentiometer and mapped from 0 cm to 60 cm. You can modify this range in the code by changing the `map()` function in the `loop()`.

### PID Performance Tuning
The system recalculates the PID parameters every 10 seconds (adjustable via `tuneInterval`). The parameters are constrained within reasonable bounds:
- **Kp**: 0.0 to 10.0
- **Ki**: 0.0 to 1.0
- **Kd**: 0.0 to 1.0

## 5. Operation

1. Upload the code to your Arduino-compatible microcontroller.
2. Adjust the setpoint using the potentiometer.
3. The motor will try to maintain the desired distance as measured by the ultrasonic sensor.
4. The PID controller will adjust itself every 10 seconds based on the average error over a 10-second window.
5. Monitor real-time data on the LCD display or through Serial output.

## 6. PID Parameters

The code implements a PID control system with three adjustable parameters:

- **Kp (Proportional)**: Controls the reaction to the current error.
- **Ki (Integral)**: Corrects for cumulative past errors.
- **Kd (Derivative)**: Predicts future errors based on the current rate of change.

The system self-tunes these parameters using performance metrics:
- **errorSum**: The cumulative error over a time window.
- **errorSquaredSum**: The sum of squared errors for variance calculation.

### Adaptive Tuning
After every 100 samples (adjustable via `windowSize`), the system adjusts the PID parameters based on the average error and error variance.

## 7. Sensor Calibration

The ultrasonic sensor measures the distance by calculating the time it takes for sound waves to reflect back. The formula used to calculate distance is:

```
distance = duration * 0.034 / 2;
```

Ensure the sensor is properly calibrated and placed in a suitable environment to get accurate readings.

## 8. Motor Control

The motor's PWM signal is adjusted based on the output of the PID controller. The output is constrained between -255 and 255 and mapped to a valid PWM range (0-255). The motor tries to maintain the distance at the setpoint, adjusting its speed as necessary.

## 9. Serial Output

To assist with debugging and system monitoring, the code outputs the following information to the Serial monitor every second:
- Setpoint
- Current Distance
- Kp, Ki, Kd values

Use the Serial Monitor in the Arduino IDE (set baud rate to 9600) to view this information in real time.

## 10. Troubleshooting

### Common Issues:
- **No LCD Output**: Double-check the I2C address (default is 0x27). Adjust it according to your hardware.
- **Motor Not Responding**: Ensure the motor is correctly connected to the PWM pin (Pin 9) and check power connections.
- **Erratic Distance Readings**: Make sure the ultrasonic sensor is properly aligned and free from obstructions.
- **PID Controller Not Tuning**: Verify that the `tuneInterval` and `windowSize` are appropriate for your system's dynamics.

### Tips:
- If the motor is oscillating too much, try reducing the `Kp` value manually.
- If the system is too slow to react, consider increasing `Kd`.