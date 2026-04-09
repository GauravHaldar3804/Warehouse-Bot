#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// ================= PID =================
float Kp = 30;
float Ki = 0;
float Kd = 15;

float prevError = 0;
float integral = 0;

// ================= LINE LOST RECOVERY =================
int lastErrorSign = 1;              // +1 or -1 based on last valid error
const float lineDetectThreshold = 0.35;
const int searchFastSpeed = 150;
const int searchSlowSpeed = 40;

// ================= SPEED =================
int baseSpeed = 130;

// ================= RUN CONTROL =================
bool isRunning = true;

// ================= DEMUX =================
int s0 = 10, s1 = 11, s2 = 12, s3 = 13;
int sensorPin = A0;

// ================= CALIBRATION =================
int sensorMin[8];
int sensorMax[8];

int weights[8] = {3,2,1,0,0,-1,-2,-3};

// ================= SETUP =================
void setup() {
  Serial.begin(9600);

  pwm.begin();
  pwm.setPWMFreq(1000);

  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);

  Serial.println("Place robot on line...");
  delay(3000);

  calibrateSensors();
}

// ================= MOTOR =================
void setMotor(int chA, int chB, int pwmVal) {
  pwmVal = constrain(pwmVal, -255, 255);

  if (pwmVal > 0) {
    pwm.setPWM(chA, 0, pwmVal * 16);
    pwm.setPWM(chB, 0, 0);
  } else {
    pwm.setPWM(chA, 0, 0);
    pwm.setPWM(chB, 0, -pwmVal * 16);
  }
}

// ================= STOP =================
void stopMotors() {
  for (int i = 0; i < 8; i++) {
    pwm.setPWM(i, 0, 0);
  }
}

// ================= SERIAL CONTROL =================
void checkCommand() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "STOP") {
      isRunning = false;
      stopMotors();
      Serial.println("STOPPED");
    }

    if (cmd == "START") {
      isRunning = true;
      Serial.println("STARTED");
    }
  }
}

// ================= SENSOR READ =================
int readSensor(int ch) {
  digitalWrite(s0, ch & 1);
  digitalWrite(s1, (ch >> 1) & 1);
  digitalWrite(s2, (ch >> 2) & 1);
  digitalWrite(s3, (ch >> 3) & 1);
  delayMicroseconds(5);
  return analogRead(sensorPin);
}

// ================= UPDATE CALIB =================
void updateCalibration() {
  for (int i = 0; i < 8; i++) {
    int val = readSensor(i);

    if (val < sensorMin[i]) sensorMin[i] = val;
    if (val > sensorMax[i]) sensorMax[i] = val;
  }
}

// ================= CALIBRATION =================
void calibrateSensors() {
  Serial.println("Calibration START");

  for (int i = 0; i < 8; i++) {
    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }

  unsigned long startTime;

  // ===== STRAFE LEFT =====
  Serial.println("Strafing LEFT...");
  startTime = millis();

  while (millis() - startTime < 2000) {
    setMotor(6,7,-120);
    setMotor(4,5,120);
    setMotor(0,1,120);
    setMotor(2,3,-120);

    updateCalibration();
    delay(10);
  }

  stopMotors();
  delay(500);

  // ===== STRAFE RIGHT =====
  Serial.println("Strafing RIGHT...");
  startTime = millis();

  while (millis() - startTime < 2000) {
    setMotor(6,7,120);
    setMotor(4,5,-120);
    setMotor(0,1,-120);
    setMotor(2,3,120);

    updateCalibration();
    delay(10);
  }

  stopMotors();

  Serial.println("Calibration DONE");

  // Print values
  for (int i = 0; i < 8; i++) {
    Serial.print("S"); Serial.print(i);
    Serial.print(" Min: "); Serial.print(sensorMin[i]);
    Serial.print(" Max: "); Serial.println(sensorMax[i]);
  }

  delay(2000);
}

// ================= NORMALIZE =================
float readNormalized(int ch) {
  int val = readSensor(ch);

  int range = sensorMax[ch] - sensorMin[ch];
  if (range < 10) return 0;

  float norm = (float)(val - sensorMin[ch]) / range;
  return constrain(norm, 0, 1);
}

// ================= LINE ERROR =================
bool getLineError(float &errorOut) {
  float numerator = 0;
  float denominator = 0;

  for (int i = 0; i < 8; i++) {
    float val = 1 - readNormalized(i);

    numerator += val * weights[i];
    denominator += val;
  }

  if (denominator < lineDetectThreshold) return false;

  errorOut = numerator / denominator;
  return true;
}

// ================= PID =================
int computePID(float error) {
  integral += error;
  float derivative = error - prevError;

  float output = Kp*error + Kd*derivative + Ki*integral;

  prevError = error;

  return (int)output;
}

// ================= LOOP =================
void loop() {

  checkCommand();

  if (!isRunning) {
    stopMotors();
    return;
  }

  float error = 0;
  int leftSpeed = 0;
  int rightSpeed = 0;

  bool lineDetected = getLineError(error);

  if (lineDetected) {
    int correction = computePID(error);

    leftSpeed  = baseSpeed + correction;
    rightSpeed = baseSpeed - correction;

    leftSpeed  = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    if (error > 0.05) {
      lastErrorSign = 1;
    } else if (error < -0.05) {
      lastErrorSign = -1;
    }
  } else {
    // Prevent PID windup while searching for line again.
    integral = 0;
    prevError = 0;

    // Turn toward the side where the line was last seen.
    if (lastErrorSign > 0) {
      leftSpeed = searchFastSpeed;
      rightSpeed = searchSlowSpeed;
    } else {
      leftSpeed = searchSlowSpeed;
      rightSpeed = searchFastSpeed;
    }
  }

  // LEFT SIDE
  setMotor(6,7,leftSpeed);
  setMotor(4,5,leftSpeed);

  // RIGHT SIDE
  setMotor(0,1,rightSpeed);
  setMotor(2,3,rightSpeed);

  if (lineDetected) {
    Serial.print("Error: ");
    Serial.println(error);
  } else {
    Serial.println("LINE LOST -> SEARCHING");
  }

  delay(10);
}
