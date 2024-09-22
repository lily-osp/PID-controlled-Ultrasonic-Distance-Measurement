# Readme

## 1. Introduction
This project implements a PID (Proportional, Integral, Derivative) control system to control a motor's speed based on distance measurements from an ultrasonic sensor. The system automatically tunes its PID parameters using the Ziegler-Nichols method during an initial tuning phase. The control values are displayed on a 20x4 I2C LCD.

The motor speed is adjusted according to the distance between the sensor and an object, allowing for precision control using feedback from the ultrasonic sensor. A potentiometer sets the desired distance (setpoint) in real-time.

## 2. Setup
- Connect an ultrasonic sensor (HC-SR04) to pins 2 (Echo) and 3 (Trig).
- Attach the motor's PWM control pin to pin 9.
- Connect a potentiometer to pin A0 to adjust the setpoint.
- The 20x4 I2C LCD is connected to display real-time information.
- A Serial monitor outputs system details for debugging and monitoring.

### Pin Configuration:
| Component          | Pin       |
|--------------------|-----------|
| Ultrasonic Sensor   | Trig: 3, Echo: 2 |
| Motor              | PWM: 9    |
| Potentiometer      | A0        |
| I2C LCD            | SDA, SCL  |

## 3. Dependencies
To run this project, you need the following libraries installed in the Arduino IDE:
- **Wire.h**: Used for I2C communication.
- **LiquidCrystal_I2C.h**: Controls the 20x4 I2C LCD display.

To install these libraries:
1. In Arduino IDE, go to **Sketch > Include Library > Manage Libraries**.
2. Search for **LiquidCrystal I2C** and install the one compatible with your LCD.

## 4. Configuration
Ensure the I2C address of your LCD is set correctly. By default, it is set to `0x27`. If your LCD has a different address, change this line accordingly:

```cpp
LiquidCrystal_I2C lcd(0x27, 20, 4);  // Change 0x27 to your LCD I2C address
```

## 5. Operation
The system starts in tuning mode, where it automatically adjusts the PID parameters for the motor control. After 30 seconds, tuning completes, and the system switches to operational mode, using the tuned PID values to control the motor speed based on the distance measured.

- **Tuning Phase**: During the first 30 seconds, the system determines optimal PID values using Ziegler-Nichols method. The LCD shows "Tuning..." during this phase.
- **Control Phase**: After tuning, the system actively controls the motor using the PID values and displays "Running" on the LCD.

The potentiometer adjusts the **setpoint** (desired distance) in real-time, and the system continuously adjusts motor speed to maintain this distance.

## 6. PID Parameters
The PID parameters are calculated automatically based on the tuning results:

- **Kp (Proportional Gain)**: Adjusts the motor speed proportionally to the error (difference between the setpoint and actual distance).
- **Ki (Integral Gain)**: Minimizes steady-state error by accumulating past errors.
- **Kd (Derivative Gain)**: Reduces overshoot by considering the rate of error change.

You can view these values on the LCD once the system transitions to control mode.

## 7. Sensor Calibration
The ultrasonic sensor measures distance by sending out sound pulses and timing how long it takes to receive an echo. The distance is calculated using the formula:

```
Distance = (Duration * 0.034) / 2
```

- Ensure the sensor is aligned properly and unobstructed for accurate distance measurement.
- The potentiometer sets the desired distance (setpoint) from **6 cm to 60 cm**.

## 8. Motor Control
The motor is controlled using PWM (Pulse Width Modulation) signals based on the PID controller’s output. The controller adjusts the PWM duty cycle (ranging from 0 to 255) to maintain the setpoint distance. 

In tuning mode, the system measures the maximum and minimum output to calculate the optimal tuning parameters (Ku and Tu). After tuning, the PID controller dynamically adjusts motor speed to minimize error.

## 9. Serial Output
The serial monitor outputs helpful debugging information:
- **Distance**: Measured distance from the sensor in centimeters.
- **Setpoint**: Desired distance as set by the potentiometer.
- **PID Values**: Current PID gains (Kp, Ki, Kd).
- **Motor PWM Output**: The PWM value sent to the motor.

Open the Serial Monitor at **9600 baud** to monitor these values during operation.

## 10. Troubleshooting
- **LCD not displaying**: Verify the I2C address of your LCD and ensure the wiring is correct.
- **Incorrect distance measurements**: Check the alignment of the ultrasonic sensor. Ensure no obstacles interfere with the sound waves.
- **Motor not responding**: Ensure the motor is connected correctly to pin 9. Test with a direct PWM signal to rule out hardware issues.
- **No Serial Output**: Confirm the baud rate is set to 9600 in both the code and the Serial Monitor.

For further debugging, increase the serial output or add additional print statements in the code to identify specific issues.