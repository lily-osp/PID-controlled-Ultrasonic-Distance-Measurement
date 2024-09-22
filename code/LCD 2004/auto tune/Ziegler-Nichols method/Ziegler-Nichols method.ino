#include <Wire.h>
#include <LiquidCrystal_I2C.h>

const int trigPin = 3;
const int echoPin = 2;
const int potentiometerPin = A0;
const int motorpin = 9;

float Kp = 0.0;
float Ki = 0.0;
float Kd = 0.0;

float setpoint = 0.0;
float error = 0.0;
float lastError = 0.0;
float integral = 0.0;

bool tuning = true;
unsigned long tuningStartTime = 0;
const unsigned long tuningDuration = 30000; // 30 seconds for tuning
float maxOutput = 0.0;
float minOutput = 1000.0;
float Ku, Tu;

LiquidCrystal_I2C lcd(0x27, 20, 4);  // Change 0x27 to your LCD I2C address

void setup() {
    Serial.begin(9600);
    lcd.begin();
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    analogWrite(motorpin, 0);
    delay(2000);
    tuningStartTime = millis();
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

void autoTunePID() {
    float distance = getDistance();
    error = setpoint - distance;
    
    float P = error;
    float output = P;
    
    if (output > maxOutput) maxOutput = output;
    if (output < minOutput) minOutput = output;
    
    if (millis() - tuningStartTime > tuningDuration) {
        tuning = false;
        Ku = 4 * (maxOutput - minOutput) / (3.14159 * (maxOutput + minOutput));
        Tu = tuningDuration / 1000.0; // Convert to seconds
        
        // Ziegler-Nichols tuning rules for PID
        Kp = 0.6 * Ku;
        Ki = 2 * Kp / Tu;
        Kd = Kp * Tu / 8;
        
        integral = 0; // Reset integral term
    }
    
    int pwmValue = map(constrain(output, -255, 255), -255, 255, 0, 255);
    analogWrite(motorpin, pwmValue);
}

void controlMotor() {
    float distance = getDistance();
    error = setpoint - distance;
    integral += error;
    float derivative = error - lastError;
    
    float output = Kp * error + Ki * integral + Kd * derivative;
    
    int pwmValue = map(constrain(output, -255, 255), -255, 255, 0, 255);
    analogWrite(motorpin, pwmValue);
    
    lastError = error;
    
    updateLCD(distance, pwmValue, output);
}

void updateLCD(float distance, int pwmValue, float pidOutput) {
    static unsigned long lastUpdateTime = 0;
    const unsigned long lcdRefreshInterval = 500;
    
    if (millis() - lastUpdateTime >= lcdRefreshInterval) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(tuning ? "Tuning..." : "Running");
        lcd.setCursor(0, 1);
        lcd.print("SP:" + String(setpoint, 1) + " D:" + String(distance, 1));
        lcd.setCursor(0, 2);
        lcd.print("Kp:" + String(Kp, 2));
        lcd.setCursor(0, 3);
        lcd.print("Ki:" + String(Ki, 3) + " Kd:" + String(Kd, 3));
        
        lastUpdateTime = millis();
    }
}

void loop() {
    int sensorValue = analogRead(potentiometerPin);
    setpoint = map(sensorValue, 0, 1023, 600, 0) / 10.0;
    
    if (tuning) {
        autoTunePID();
    } else {
        controlMotor();
    }
    
    delay(100);
}