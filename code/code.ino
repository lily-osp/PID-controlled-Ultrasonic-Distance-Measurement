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

void setup() {
  Serial.begin(9600);             
  pinMode(trigPin, OUTPUT);      
  pinMode(echoPin, INPUT);
  analogWrite(motorpin, 0);
  delay(2000);
}

void loop() {
  // Ultrasonic Sensor Part
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  float distance_cm = duration * 0.034 / 2;

  // Potentiometer and MOTOR Part
  int sensorValue = analogRead(potentiometerPin);
  setpoint = map(sensorValue, 0, 1023, 600, 0);  // Adjust the setpoint range based on your needs
  setpoint = setpoint/10;

  error = setpoint - distance_cm;
  integral += error;
  float derivative = error - lastError;

  // Calculate PID output
  float output = Kp * error + Ki * integral + Kd * derivative;

  // Map the PID output to PWM range
  int pwmValue = map(output, 255, -255, 0, 255);
  pwmValue = constrain(pwmValue, 143, 154);

  analogWrite(motorpin, pwmValue);
  Serial.println("Setpoint: " + String(setpoint) + "\tDistance: " + String(distance_cm) + "\tPWM Value: " + String(pwmValue) + "\tPID Value: " + String(output));

  lastError = error;

  delay(100); 
}
