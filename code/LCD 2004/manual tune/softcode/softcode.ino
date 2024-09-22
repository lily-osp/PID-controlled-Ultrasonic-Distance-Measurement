#include <Wire.h>
#include <LiquidCrystal_I2C.h>

const int trigPin = 3;
const int echoPin = 2;
const int potentiometerPin = A0;
const int motorpin = 9;
const int kpPin = A1;  // New potentiometer for Kp
const int kiPin = A2;  // New potentiometer for Ki
const int kdPin = A3;  // New potentiometer for Kd

float Kp = 1.05;  // Initial values, will be adjusted
float Ki = 0.026;
float Kd = 0.095;

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

float getDistance() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH);
    return duration * 0.034 / 2;
}

void updatePIDValues() {
    Kp = map(analogRead(kpPin), 0, 1023, 0, 500) / 100.0;  // Range: 0.00 to 5.00
    Ki = map(analogRead(kiPin), 0, 1023, 0, 100) / 1000.0;  // Range: 0.000 to 0.100
    Kd = map(analogRead(kdPin), 0, 1023, 0, 200) / 1000.0;  // Range: 0.000 to 0.200
}

void controlMotor() {
    updatePIDValues();
    
    int sensorValue = analogRead(potentiometerPin);
    setpoint = map(sensorValue, 0, 1023, 600, 0) / 10.0;  // Adjust the setpoint range based on your needs
    
    float distance = getDistance();
    error = setpoint - distance;
    integral += error;
    float derivative = error - lastError;
    
    // Calculate PID output
    float output = Kp * error + Ki * integral + Kd * derivative;
    
    // Map the PID output to PWM range
    int pwmValue = map(constrain(output, -255, 255), -255, 255, 143, 154);
    analogWrite(motorpin, pwmValue);
    
    lastError = error;
    
    // Display on Serial Monitor
    Serial.println("Setpoint: " + String(setpoint) + "\tDistance: " + String(distance) +
                    "\tPWM Value: " + String(pwmValue) + "\tPID Output: " + String(output));
    
    // Update the LCD display
    updateLCD(distance, pwmValue, output);
}

void updateLCD(float distance, int pwmValue, float pidOutput) {
    static unsigned long lastUpdateTime = 0;
    const unsigned long lcdRefreshInterval = 500;  // Set your desired refresh interval in milliseconds
    
    if (millis() - lastUpdateTime >= lcdRefreshInterval) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("SP:" + String(setpoint, 1) + " D:" + String(distance, 1));
        lcd.setCursor(0, 1);
        lcd.print("PWM:" + String(pwmValue) + " PID:" + String(pidOutput, 1));
        lcd.setCursor(0, 2);
        lcd.print("Kp:" + String(Kp, 2));
        lcd.setCursor(0, 3);
        lcd.print("Ki:" + String(Ki, 3) + " Kd:" + String(Kd, 3));
        
        lastUpdateTime = millis();
    }
}

void loop() {
    controlMotor();
    delay(100);
}