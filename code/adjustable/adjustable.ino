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
const long interval = 100;  // Interval in milliseconds

void setup() {
    Serial.begin(9600);
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    pinMode(KpPotPin, INPUT);
    pinMode(KiPotPin, INPUT);
    pinMode(KdPotPin, INPUT);
    analogWrite(motorpin, 0);
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

    Serial.println("Setpoint: " + String(setpoint) + "\tDistance: " + String(distance_cm));
}

void readPotentiometers() {
    // Read values from potentiometers for Kp, Ki, and Kd and treat those as float
    Kp = map(analogRead(KpPotPin), 0, 1023, 0, 200) / 100.000;
    Ki = map(analogRead(KiPotPin), 0, 1023, 0, 200) / 100.000;
    Kd = map(analogRead(KdPotPin), 0, 1023, 0, 200) / 100.000;

    Serial.println("Kp: " + String(Kp) + "\tKi: " + String(Ki) + "\tKd: " + String(Kd));
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
    Serial.println("\tPWM Value: " + String(pwmValue) + "\tPID Value: " + String(output));

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
