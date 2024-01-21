#include <Wire.h>
#include <LiquidCrystal_I2C.h>

const int trigPin = 3;
const int echoPin = 2;
const int potentiometerPin = A0;
const int motorpin = 9;

const float Kp = 1.05;                 // Proportional gain
const float Ki = 0.026;                // Integral gain
const float Kd = 0.095;                // Derivative gain

float setpoint = 0.0;
float error = 0.0;
float lastError = 0.0;
float integral = 0.0;

LiquidCrystal_I2C lcd(0x27, 20, 4);  // Change 0x27 to your LCD I2C address

void setup() {
    Serial.begin(9600);
    lcd.begin(); 
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    analogWrite(motorpin, 0);
    delay(2000);
}

// Ultrasonic Sensor Function
float getDistance() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH);
    return duration * 0.034 / 2;
}

// Potentiometer and Motor Function
void controlMotor() {
    int sensorValue = analogRead(potentiometerPin);
    setpoint = map(sensorValue, 0, 1023, 600, 0);  // Adjust the setpoint range based on your needs
    setpoint = setpoint / 10;

    error = setpoint - getDistance();
    integral += error;
    float derivative = error - lastError;

    // Calculate PID output
    float output = Kp * error + Ki * integral + Kd * derivative;

    // Map the PID output to PWM range
    int pwmValue = map(output, 255, -255, 0, 255);
    pwmValue = constrain(pwmValue, 143, 154);

    analogWrite(motorpin, pwmValue);

    lastError = error;

    // Display on Serial Monitor
    Serial.println("Setpoint: " + String(setpoint) + "\tDistance: " + String(getDistance()) +
                    "\tPWM Value: " + String(pwmValue) + "\tPID Value: " + String(output));

    // Update the LCD display with a delay
    updateLCD();
}

// Function to update the LCD display
void updateLCD() {
    static unsigned long lastUpdateTime = 0;
    const unsigned long lcdRefreshInterval = 500;  // Set your desired refresh interval in milliseconds

    if (millis() - lastUpdateTime >= lcdRefreshInterval) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Setpoint: " + String(setpoint));
        lcd.setCursor(0, 1);
        lcd.print("Distance: " + String(getDistance()));
        lcd.setCursor(0, 2);
        lcd.print("PWM Value: " + String(analogRead(motorpin)));
        lcd.setCursor(0, 3);
        lcd.print("PID Value: " + String(Kp * error + Ki * integral + Kd * (error - lastError)));

        lastUpdateTime = millis();
    }
}

void loop() {
    controlMotor();
    delay(100);
}
