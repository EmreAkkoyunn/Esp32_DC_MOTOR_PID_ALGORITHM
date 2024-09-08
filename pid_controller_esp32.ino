#define RPWM_PIN 18
#define LPWM_PIN 19


#define encoderPinA 14
#define encoderPinB 27

volatile long encoderCount = 0; 
unsigned long previousTime = 0;
float ePrevious = 0;
float eIntegral = 0;

const float kp = 0.62;    // Proportional gain
const float kd = 0.04;    // Derivative gain
const float ki = 0.99;    // Integral gain
const float set_point_rpm = 180.0;  

const int encoderPulsePerRevolution = 1172;  
const unsigned long measurementInterval = 1000;  

void setup() {
  Serial.begin(9600);  // Initialize serial communication

  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), handleEncoder, RISING);  // Set up encoder interrupt

  previousTime = millis();  // Initialize previous time
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - previousTime >= measurementInterval) {
    float current_rpm = calculateRPM();  // Calculate current RPM
    float u = pidController(set_point_rpm, kp, kd, ki, current_rpm);  // Calculate PID output

    // Limit the PID output
    float pwr = constrain(u, 0, 255);

    // If PID output is negative, reduce the speed
    if (u < 0) {
      pwr = constrain(255 + u, 0, 255); // Add the negative value to reduce speed
    }

    moveMotor(RPWM_PIN, LPWM_PIN, pwr, 1);  // Move motor forward

    // Send data in the correct format for the Serial Plotter
    Serial.print("SP:");  // SP: Set Point RPM
    Serial.print(set_point_rpm);
    Serial.print(",RPM:"); // RPM: Current RPM
    Serial.print(current_rpm);
    Serial.print(",PID:"); // PID: PID Output
    Serial.println(u);

    previousTime = currentTime;  // Update the time
  }

  delay(10);  // Reduce delay to make the loop faster
}

void handleEncoder() {
  if (digitalRead(encoderPinA) > digitalRead(encoderPinB)) {
    encoderCount++;  // Increment encoder count
  } else {
    encoderCount--;  // Decrement encoder count
  }
}

void moveMotor(int rpwmPin, int lpwmPin, int speed, int dir) {
  analogWrite(rpwmPin, speed);  // Set motor speed
  analogWrite(lpwmPin, 0);      // Ensure the other direction pin is low
}

float calculateRPM() {
  static long previousCount = 0;
  unsigned long elapsedTime = millis() - previousTime;

  // Skip RPM calculation if elapsed time is too short
  if (elapsedTime == 0) {
    return 0;
  }

  float rpm = ((encoderCount - previousCount) / (float)encoderPulsePerRevolution) * (60000.0 / elapsedTime);

  previousCount = encoderCount;  // Update previous count
  return rpm;
}

float pidController(float target_rpm, float kp, float kd, float ki, float current_rpm) {
  static unsigned long lastPIDTime = millis();
  unsigned long currentTime = millis();
  float deltaT = ((float)(currentTime - lastPIDTime)) / 1000.0;  // Time in seconds

  if (deltaT <= 0) {
    return 0;
  }

  float e = target_rpm - current_rpm;  // Error between setpoint and actual RPM
  float eDerivative = (e - ePrevious) / deltaT;  // Error derivative
  eIntegral += e * deltaT;  // Error integral

  // Anti-windup: limit the integral term
  eIntegral = constrain(eIntegral, -255, 255);

  float u = (kp * e) + (kd * eDerivative) + (ki * eIntegral);  // PID control output

  ePrevious = e;  // Update previous error
  lastPIDTime = currentTime;  // Update time

  return u;  // Return the PID output
}

float x_contrain(float value, float min, float max) {
  if (value > max) {
    value = max;  // Limit value to max
  } else if (value < max) {
    value = min;  // Limit value to min
  }
  return value;
}
