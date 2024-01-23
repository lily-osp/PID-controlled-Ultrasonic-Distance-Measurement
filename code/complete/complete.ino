#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);  // Change the address if your LCD has a different address

const int trigPin = 3;
const int echoPin = 2;
const int SetpointPin = A0;        // Potentiometer for set point
const int KpPotPin = A1;           // Potentiometer for Kp
const int KiPotPin = A2;           // Potentiometer for Ki
const int KdPotPin = A3;           // Potentiometer for Kd
const int motorpin = 9;

float Kp, Ki, Kd;                  // Proportional, Integral, and Derivative gains
float setpoint = 0.00;
float error = 0.00;
float lastError = 0.00;
float integral = 0.00;
float distance_cm = 0.00;

unsigned long previousMillis = 0;
unsigned long lcdMillis = 0;  // Variable to track LCD update time
const long interval = 60;  // Interval in milliseconds
const long displayInterval = 500;  // Display interval in milliseconds

void setup() {
    Serial.begin(9600);
    lcd.begin();
    lcd.backlight();

    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    pinMode(KpPotPin, INPUT);
    pinMode(KiPotPin, INPUT);
    pinMode(KdPotPin, INPUT);
    analogWrite(motorpin, 0);

    // Display the welcome message on the LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Welcome to");
    lcd.setCursor(0, 1);
    lcd.print("Your Project");

    delay(2000);
}

void displayOnLCD() {
    // Display setpoint, actual position, PWM Value, and PID Value on the LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Setpoint: " + String(setpoint));
    lcd.setCursor(0, 1);
    lcd.print("Distance: " + String(distance_cm));
    lcd.setCursor(0, 2);
    lcd.print("PWM: " + String(map(output, 255, -255, 0, 255)));
    lcd.setCursor(0, 3);
    lcd.print("PID: " + String(output));
}

void readSensors() {
    // Ultrasonic Sensor Part
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH);
    distance_cm = duration * 0.034 / 2;

    // Setpoint and MOTOR Part
    setpoint = map(analogRead(SetpointPin), 0, 1023, 600, 0) / 10.00;  // Treat setpoint as float
}

void readPotentiometers() {
    // Read values from potentiometers for Kp, Ki, and Kd and treat those as float
    float newKp = map(analogRead(KpPotPin), 0, 1023, 0, 200) / 100.000;
    float newKi = map(analogRead(KiPotPin), 0, 1023, 0, 200) / 100.000;
    float newKd = map(analogRead(KdPotPin), 0, 1023, 0, 200) / 100.000;

    // Check if there is a change in any of the potentiometer values
    if (newKp != Kp || newKi != Ki || newKd != Kd) {
        Kp = newKp;
        Ki = newKi;
        Kd = newKd;

        // Display Kp, Ki, and Kd setting for 5 seconds
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Kp Ki Kd setting");

        lcd.setCursor(0, 1);
        lcd.print("Kp: " + String(Kp));

        lcd.setCursor(0, 2);
        lcd.print("Ki: " + String(Ki));

        lcd.setCursor(0, 3);
        lcd.print("Kd: " + String(Kd));

        delay(5000);  // Display the settings for 5 seconds
    }
}

void calculatePID() {
    error = setpoint - distance_cm;
    integral += error;
    float derivative = error - lastError;

    // Calculate PID output
    float output = Kp * error + Ki * integral + Kd * derivative;

    // Map the PID output to PWM range
    int pwmValue = map(output, 255, -255, 0, 255);
    pwmValue = constrain(pwmValue, 143, 154);

    analogWrite(motorpin, pwmValue);

    // Display on the LCD at a 500ms interval
    unsigned long currentMillis = millis();
    if (currentMillis - lcdMillis >= displayInterval) {
        displayOnLCD();
        lcdMillis = currentMillis;
    }

    lastError = error;
}

void loop() {
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
        readSensors();
        readPotentiometers();
        calculatePID();

        previousMillis = currentMillis;
    }
}
