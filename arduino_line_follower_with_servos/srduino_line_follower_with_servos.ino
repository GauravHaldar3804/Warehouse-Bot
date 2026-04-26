#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// ================= PID =================
float Kp = 50;
float Ki = 0;
float Kd = 15;

float prevError = 0;
float integral = 0;

// ================= LINE LOST RECOVERY =================
int lastErrorSign = 1;              // +1 or -1 based on last valid error
const int lineLostNormThreshold = 600;
const int searchFastSpeed = 255;
const int searchSlowSpeed = -255;

// ================= SPEED =================
int baseSpeed = 180;

// ================= RUN CONTROL =================
bool isRunning = false;
bool calibrationDone = false;

enum CommandMode {
  MODE_AUTO,
  MODE_MANUAL
};

CommandMode controlMode = MODE_AUTO;

enum ManualCommand {
  CMD_NONE,
  CMD_STRAIGHT,
  CMD_LEFT,
  CMD_RIGHT,
  CMD_UTURN
};

ManualCommand manualCommand = CMD_NONE;

const int manualStraightSpeed = 150;
const int manualTurnSpeed = 170;
const unsigned long leftRightTurnDurationMs = 2000;
const unsigned long uturnDurationMs = 4000;

unsigned long manualCommandStartMs = 0;
unsigned long manualCommandDurationMs = 0;

// ================= ENCODERS (QUADRATURE) =================
const uint8_t encRFPinA = 2;
const uint8_t encRFPinB = 3;
const uint8_t encRBPinA = 4;
const uint8_t encRBPinB = 5;
const uint8_t encLBPinA = 6;
const uint8_t encLBPinB = 7;
const uint8_t encLFPinA = 8;
const uint8_t encLFPinB = 9;

long encRFCount = 0;
long encRBCount = 0;
long encLBCount = 0;
long encLFCount = 0;

uint8_t encRFPrevState = 0;
uint8_t encRBPrevState = 0;
uint8_t encLBPrevState = 0;
uint8_t encLFPrevState = 0;

unsigned long lastEncoderPublishMs = 0;
const unsigned long encoderPublishIntervalMs = 10;  // 100 Hz encoder stream
unsigned long lastStatusPrintMs = 0;
const unsigned long statusPrintIntervalMs = 200;

int8_t quadratureDelta(uint8_t prevState, uint8_t currState) {
  // Index = (prevAB << 2) | currAB
  // Valid forward transitions => +1, reverse => -1, invalid/bounce => 0
  static const int8_t table[16] = {
    0, -1,  1,  0,
    1,  0,  0, -1,
   -1,  0,  0,  1,
    0,  1, -1,  0
  };
  return table[(prevState << 2) | currState];
}

uint8_t readEncoderAB(uint8_t pinA, uint8_t pinB) {
  uint8_t a = (uint8_t)digitalRead(pinA);
  uint8_t b = (uint8_t)digitalRead(pinB);
  return (uint8_t)((a << 1) | b);
}

void updateOneEncoder(long &count, uint8_t pinA, uint8_t pinB, uint8_t &prevState) {
  uint8_t currState = readEncoderAB(pinA, pinB);
  int8_t delta = quadratureDelta(prevState, currState);
  count += delta;
  prevState = currState;
}

void updateEncoders() {
  updateOneEncoder(encRFCount, encRFPinA, encRFPinB, encRFPrevState);
  updateOneEncoder(encRBCount, encRBPinA, encRBPinB, encRBPrevState);
  updateOneEncoder(encLBCount, encLBPinA, encLBPinB, encLBPrevState);
  updateOneEncoder(encLFCount, encLFPinA, encLFPinB, encLFPrevState);
}

void publishEncodersIfDue() {
  unsigned long now = millis();
  if ((now - lastEncoderPublishMs) < encoderPublishIntervalMs) return;
  lastEncoderPublishMs = now;

  // Prefix with ENC so Raspberry Pi parser can filter robustly.
  // Format: ENC,<rf>,<rb>,<lb>,<lf>
  Serial.print("ENC,");
  Serial.print(encRFCount);
  Serial.print(',');
  Serial.print(encRBCount);
  Serial.print(',');
  Serial.print(encLBCount);
  Serial.print(',');
  Serial.println(encLFCount);
}

void startTimedManualCommand(ManualCommand cmd, unsigned long durationMs) {
  controlMode = MODE_MANUAL;
  manualCommand = cmd;
  manualCommandStartMs = millis();
  manualCommandDurationMs = durationMs;
}

void applyDrive(int leftSpeed, int rightSpeed) {
  // LEFT SIDE
  setMotor(6, 7, leftSpeed);
  setMotor(4, 5, leftSpeed);

  // RIGHT SIDE
  setMotor(0, 1, rightSpeed);
  setMotor(2, 3, rightSpeed);
}

void applyManualCommand() {
  switch (manualCommand) {
    case CMD_STRAIGHT:
      applyDrive(manualStraightSpeed, manualStraightSpeed);
      break;
    case CMD_LEFT:
      // In-place left turn
      applyDrive(-manualTurnSpeed, manualTurnSpeed);
      break;
    case CMD_RIGHT:
      // In-place right turn
      applyDrive(manualTurnSpeed, -manualTurnSpeed);
      break;
    case CMD_UTURN:
      // U-turn using a strong in-place rotation
      applyDrive(manualTurnSpeed, -manualTurnSpeed);
      break;
    case CMD_NONE:
    default:
      stopMotors();
      break;
  }
}

// ================= DEMUX =================
int s0 = 10, s1 = 11, s2 = 12, s3 = 13;
int sensorPin = A0;

// ================= CALIBRATION =================
int sensorMin[8];
int sensorMax[8];

int weights[8] = {3,2,1,0,0,-1,-2,-3};

// ================= CAMERA PAN/TILT SERVOS =================
const uint8_t panServoChannel = 8;
const uint8_t tiltServoChannel = 9;

const int panLockDeg = 90;
const int tiltDownDeg = 0;
const int tiltFrontDeg = 90;

// PCA9685 pulse counts for typical servos at 50 Hz.
const uint16_t servoPulseMin = 110;
const uint16_t servoPulseMax = 510;

bool obstacleDetected = false;

uint16_t angleToPulse(int angleDeg) {
  angleDeg = constrain(angleDeg, 0, 180);
  long pulse = map(angleDeg, 0, 180, servoPulseMin, servoPulseMax);
  return (uint16_t)pulse;
}

void setServoAngle(uint8_t channel, int angleDeg) {
  pwm.setPWM(channel, 0, angleToPulse(angleDeg));
}

void updateCameraServos() {
  // Pan remains locked straight ahead.
  setServoAngle(panServoChannel, panLockDeg);
  // Tilt looks front when obstacle exists, else looks down.
  setServoAngle(tiltServoChannel, obstacleDetected ? tiltFrontDeg : tiltDownDeg);
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(2);

  pwm.begin();
  // Servo channels on PCA9685 require 50 Hz timing.
  pwm.setPWMFreq(50);

  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);

  pinMode(encRFPinA, INPUT_PULLUP);
  pinMode(encRFPinB, INPUT_PULLUP);
  pinMode(encRBPinA, INPUT_PULLUP);
  pinMode(encRBPinB, INPUT_PULLUP);
  pinMode(encLBPinA, INPUT_PULLUP);
  pinMode(encLBPinB, INPUT_PULLUP);
  pinMode(encLFPinA, INPUT_PULLUP);
  pinMode(encLFPinB, INPUT_PULLUP);

  encRFPrevState = readEncoderAB(encRFPinA, encRFPinB);
  encRBPrevState = readEncoderAB(encRBPinA, encRBPinB);
  encLBPrevState = readEncoderAB(encLBPinA, encLBPinB);
  encLFPrevState = readEncoderAB(encLFPinA, encLFPinB);

  updateCameraServos();

  Serial.println("Ready. Send START to run first calibration and begin.");
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
    cmd.toUpperCase();

    if (cmd == "STOP") {
      obstacleDetected = true;
      updateCameraServos();
      isRunning = false;
      controlMode = MODE_MANUAL;
      manualCommand = CMD_NONE;
      manualCommandDurationMs = 0;
      stopMotors();
      Serial.println("STOPPED");
      return;
    }

    if (cmd == "START") {
      obstacleDetected = false;
      updateCameraServos();
      if (!calibrationDone) {
        Serial.println("FIRST START -> CALIBRATING");
        Serial.println("Place robot on line...");
        delay(3000);
        calibrateSensors();
        calibrationDone = true;
      }
      isRunning = true;
      controlMode = MODE_AUTO;
      manualCommand = CMD_NONE;
      manualCommandDurationMs = 0;
      Serial.println("STARTED");
      return;
    }

    if (cmd == "OBSTACLE") {
      obstacleDetected = true;
      updateCameraServos();
      Serial.println("CAMERA TILT FRONT (OBSTACLE)");
      return;
    }

    if (cmd == "CLEAR") {
      obstacleDetected = false;
      updateCameraServos();
      Serial.println("CAMERA TILT DOWN (CLEAR)");
      return;
    }

    if (cmd == "ENC_RESET") {
      encRFCount = 0;
      encRBCount = 0;
      encLBCount = 0;
      encLFCount = 0;
      Serial.println("ENCODERS RESET");
      return;
    }

    // Strict START/STOP gate: ignore motion commands until START is received.
    if (!isRunning) {
      Serial.println("IGNORED (STOPPED): send START first");
      return;
    }

    if (cmd == "STRAIGHT" || cmd == "FORWARD") {
      isRunning = true;
      controlMode = MODE_AUTO;
      manualCommand = CMD_NONE;
      manualCommandDurationMs = 0;
      Serial.println("MANUAL STRAIGHT");
      return;
    }

    if (cmd == "LEFT") {
      isRunning = true;
      startTimedManualCommand(CMD_LEFT, leftRightTurnDurationMs);
      Serial.println("MANUAL LEFT");
      return;
    }

    if (cmd == "RIGHT" || cmd == "TURN") {
      isRunning = true;
      startTimedManualCommand(CMD_RIGHT, leftRightTurnDurationMs);
      Serial.println("MANUAL RIGHT");
      return;
    }

    if (cmd == "UTURN") {
      isRunning = true;
      startTimedManualCommand(CMD_UTURN, uturnDurationMs);
      Serial.println("MANUAL UTURN");
      return;
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
    updateEncoders();
    publishEncodersIfDue();
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
    updateEncoders();
    publishEncodersIfDue();
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
  bool allBelowThreshold = true;

  for (int i = 0; i < 8; i++) {
    float val = 1 - readNormalized(i);
    int norm1000 = (int)(val * 1000.0);
    if (norm1000 >= lineLostNormThreshold) {
      allBelowThreshold = false;
    }

    numerator += val * weights[i];
    denominator += val;
  }

  if (allBelowThreshold) return false;
  if (denominator <= 0.0001) return false;

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

  updateEncoders();
  publishEncodersIfDue();
  checkCommand();

  if (!isRunning) {
    stopMotors();
    delay(10);
    return;
  }

  // In manual mode, ignore line sensors and execute command directly.
  if (controlMode == MODE_MANUAL) {
    applyManualCommand();

    if (
      manualCommandDurationMs > 0 &&
      (millis() - manualCommandStartMs) >= manualCommandDurationMs
    ) {
      controlMode = MODE_AUTO;
      manualCommand = CMD_NONE;
      manualCommandDurationMs = 0;
      Serial.println("MANUAL TURN DONE -> AUTO MODE");
    }

    delay(10);
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

  applyDrive(leftSpeed, rightSpeed);

  if (lineDetected) {
    if (millis() - lastStatusPrintMs >= statusPrintIntervalMs) {
      lastStatusPrintMs = millis();
      Serial.print("Error: ");
      Serial.println(error);
    }
  } else {
    if (millis() - lastStatusPrintMs >= statusPrintIntervalMs) {
      lastStatusPrintMs = millis();
      Serial.println("LINE LOST -> SEARCHING");
    }
  }

  delay(10);
}
