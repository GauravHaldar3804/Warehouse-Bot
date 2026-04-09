#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// ================= PID =================
float Kp = 25;
float Ki = 0;
float Kd = 15;

float prevError = 0;
float integral = 0;

// ================= SPEED =================
int baseSpeed = 120;

// ================= RUN CONTROL =================
bool isRunning = true;

// ================= DEMUX =================
int s0 = 10, s1 = 11, s2 = 12, s3 = 13;
int sensorPin = A0;

// ================= CALIBRATION =================
int sensorMin[8];
int sensorMax[8];

// ================= SENSOR =================
int weights[8] = {3,2,1,0,0,-1,-2,-3};
float sensorVals[8];

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

void stopMotors() {
  for (int i = 0; i < 8; i++) pwm.setPWM(i, 0, 0);
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

// ================= CALIBRATION =================
void updateCalibration() {
  for (int i = 0; i < 8; i++) {
    int val = readSensor(i);

    if (val < sensorMin[i]) sensorMin[i] = val;
    if (val > sensorMax[i]) sensorMax[i] = val;
  }
}

void calibrateSensors() {
  Serial.println("Calibration START");

  for (int i = 0; i < 8; i++) {
    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }

  for (int cycle = 0; cycle < 2; cycle++) {

    // LEFT STRAFE
    Serial.println("LEFT...");
    unsigned long t = millis();
    while (millis() - t < 2500) {
      setMotor(6,7,-120);
      setMotor(4,5,120);
      setMotor(0,1,120);
      setMotor(2,3,-120);

      updateCalibration();
      delay(10);
    }

    stopMotors();
    delay(400);

    // RIGHT STRAFE
    Serial.println("RIGHT...");
    t = millis();
    while (millis() - t < 2500) {
      setMotor(6,7,120);
      setMotor(4,5,-120);
      setMotor(0,1,-120);
      setMotor(2,3,120);

      updateCalibration();
      delay(10);
    }

    stopMotors();
    delay(400);
  }

  Serial.println("Calibration DONE");

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
float getLineError() {
  float numerator = 0;
  float denominator = 0;

  for (int i = 0; i < 8; i++) {
    sensorVals[i] = 1 - readNormalized(i);

    numerator += sensorVals[i] * weights[i];
    denominator += sensorVals[i];
  }

  // Line lost → continue turning
  if (denominator == 0) {
    return prevError > 0 ? 3 : -3;
  }

  return numerator / denominator;
}

// ================= PID =================
int computePID(float error) {
  integral += error;
  float derivative = error - prevError;

  float output = Kp*error + Kd*derivative + Ki*integral;

  prevError = error;

  return (int)output;
}

// ================= TURN FUNCTIONS =================
void turnLeft() {
  Serial.println("LEFT TURN");

  while (true) {
    setMotor(6,7,-120);
    setMotor(4,5,-120);
    setMotor(0,1,120);
    setMotor(2,3,120);

    float center = (1 - readNormalized(3)) + (1 - readNormalized(4));

    if (center > 0.8) break;
  }

  stopMotors();
}

void turnRight() {
  Serial.println("RIGHT TURN");

  while (true) {
    setMotor(6,7,120);
    setMotor(4,5,120);
    setMotor(0,1,-120);
    setMotor(2,3,-120);

    float center = (1 - readNormalized(3)) + (1 - readNormalized(4));

    if (center > 0.8) break;
  }

  stopMotors();
}

// ================= LOOP =================
void loop() {

  checkCommand();

  if (!isRunning) {
    stopMotors();
    return;
  }

  float error = getLineError();

  // ===== TURN DETECTION =====
  if (sensorVals[0] > 0.8 && sensorVals[1] > 0.8 && sensorVals[2] > 0.8) {
    turnLeft();
    return;
  }

  if (sensorVals[5] > 0.8 && sensorVals[6] > 0.8 && sensorVals[7] > 0.8) {
    turnRight();
    return;
  }

  // ===== PID =====
  int correction = computePID(error);

  int leftSpeed  = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  leftSpeed  = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // LEFT SIDE
  setMotor(6,7,leftSpeed);
  setMotor(4,5,leftSpeed);

  // RIGHT SIDE
  setMotor(0,1,rightSpeed);
  setMotor(2,3,rightSpeed);

  Serial.print("Error: ");
  Serial.println(error);

  delay(10);
}
