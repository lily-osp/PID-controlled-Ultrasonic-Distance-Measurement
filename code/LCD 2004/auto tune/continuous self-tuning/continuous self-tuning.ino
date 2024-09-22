#include <Wire.h>
#include <LiquidCrystal_I2C.h>

const int trigPin = 3;
const int echoPin = 2;
const int potentiometerPin = A0;
const int motorpin = 9;

float Kp = 1.0;
float Ki = 0.0;
float Kd = 0.0;

float setpoint = 0.0;
float error = 0.0;
float lastError = 0.0;
float integral = 0.0;

const int windowSize = 100;
float errorSum = 0.0;
float errorSquaredSum = 0.0;
int sampleCount = 0;

unsigned long lastTuneTime = 0;
const unsigned long tuneInterval = 10000; // Tune every 10 seconds

const float learningRate = 0.01;

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

void updatePerformanceMetrics(float error) {
    errorSum += error;
    errorSquaredSum += error * error;
    sampleCount++;

    if (sampleCount >= windowSize) {
        float avgError = errorSum / windowSize;
        float errorVariance = (errorSquaredSum / windowSize) - (avgError * avgError);
        
        // Adjust PID parameters based on performance metrics
        Kp += learningRate * avgError;
        Ki += learningRate * integral;
        Kd += learningRate * errorVariance;

        // Ensure PID parameters stay within reasonable bounds
        Kp = constrain(Kp, 0.0, 10.0);
        Ki = constrain(Ki, 0.0, 1.0);
        Kd = constrain(Kd, 0.0, 1.0);

        // Reset metrics for next window
        errorSum = 0.0;
        errorSquaredSum = 0.0;
        sampleCount = 0;
    }
}

float controlMotor() {
    float distance = getDistance();
    error = setpoint - distance;
    integral += error;
    float derivative = error - lastError;
    
    float output = Kp * error + Ki * integral + Kd * derivative;
    
    int pwmValue = map(constrain(output, -255, 255), -255, 255, 0, 255);
    analogWrite(motorpin, pwmValue);
    
    lastError = error;
    
    updatePerformanceMetrics(error);
    
    return distance;
}

void updateLCD(float distance) {
    static unsigned long lastUpdateTime = 0;
    const unsigned long lcdRefreshInterval = 500;
    
    if (millis() - lastUpdateTime >= lcdRefreshInterval) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("SP:" + String(setpoint, 1) + " D:" + String(distance, 1));
        lcd.setCursor(0, 1);
        lcd.print("Kp:" + String(Kp, 2));
        lcd.setCursor(0, 2);
        lcd.print("Ki:" + String(Ki, 3));
        lcd.setCursor(0, 3);
        lcd.print("Kd:" + String(Kd, 3));
        
        lastUpdateTime = millis();
    }
}

void loop() {
    int sensorValue = analogRead(potentiometerPin);
    setpoint = map(sensorValue, 0, 1023, 600, 0) / 10.0;
    
    float distance = controlMotor();
    updateLCD(distance);
    
    // Print debug info to Serial every second
    static unsigned long lastPrintTime = 0;
    if (millis() - lastPrintTime >= 1000) {
        Serial.println("Setpoint: " + String(setpoint) + ", Distance: " + String(distance) + ", Kp: " + String(Kp) + ", Ki: " + String(Ki) + ", Kd: " + String(Kd));
        lastPrintTime = millis();
    }
    
    delay(10);
}