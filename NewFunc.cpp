#include <Wire.h>
#include <FastLED.h>
#include <Servo.h>

#define NUM_LEDS 2
#define PIN_RBGLED 4
#define PWR_R 5
#define PWR_L 6
#define MTR_R 8
#define MTR_L 7
#define SERVO 10
#define MTR_ENABLE 3
#define US_OUT 13
#define US_IN 12
#define BUTTON 2
#define LEFT A2
#define MIDDLE A1
#define RIGHT A0
#define L_TH 450
#define M_TH 75
#define R_TH 85

#define GYRO 0x68
#define SCAN_POINTS 7

#define DETECTION_RANGE 15

#define READINGS_PER_ANGLE 4

// How many ms between readings at each angle during detection sweep
#define READING_DELAY 150

// Minimum delta (cm) across readings at an angle to count as a moving object
#define MOVEMENT_THRESHOLD 5

// Stop this close to the object being followed
#define FOLLOW_STOP_DIST 12

// How slow the follow sweep runs (lower = slower, more time per angle)
#define SERVO_SWEEP_SPEED 0.05

CRGB leds[NUM_LEDS];
Servo scanServo;

int lastTurn = 1;
int numerOfLefts = 0;
int numerOfRights = 0;

float angle = 0;
float gyroOffset = 0;
unsigned long lastTime = 0;

int scanAngles[SCAN_POINTS]     = {-90, -60, -30, 0, 30, 60, 90};
int scanDistances[SCAN_POINTS]  = {999, 999, 999, 999, 999, 999, 999};
int prevDistances[SCAN_POINTS]  = {999, 999, 999, 999, 999, 999, 999};
int memory[SCAN_POINTS]         = {0};

float servoPhase = 0.0;
unsigned long lastServoUpdate = 0;

bool following = false;
int followAngleIndex = 3;

// ===================== MOTORS =====================

void setupMotors() {
  pinMode(PWR_L, OUTPUT);
  pinMode(PWR_R, OUTPUT);
  pinMode(MTR_L, OUTPUT);
  pinMode(MTR_R, OUTPUT);
  pinMode(MTR_ENABLE, OUTPUT);
  digitalWrite(MTR_ENABLE, HIGH);
}

void stopMotors() {
  analogWrite(PWR_L, 0);
  analogWrite(PWR_R, 0);
  digitalWrite(MTR_L, LOW);
  digitalWrite(MTR_R, LOW);
}

void moveMotors(int leftSpeed, int rightSpeed) {
  digitalWrite(MTR_L, leftSpeed  >= 0 ? HIGH : LOW);
  digitalWrite(MTR_R, rightSpeed >= 0 ? HIGH : LOW);
  analogWrite(PWR_L, constrain(abs(leftSpeed),  0, 255));
  analogWrite(PWR_R, constrain(abs(rightSpeed), 0, 255));
}

void driveStraight(int speed) {
  float correction = constrain(angle * 12.0, -70, 70);
  int left  = constrain(speed - correction + 5, 0, 255);
  int right = constrain(speed + correction - 5, 0, 255);
  moveMotors(left, right);
  angle *= 0.97;
}

// ===================== GYRO =====================

bool setupGyro() {
  Wire.begin();
  Wire.beginTransmission(GYRO);
  Wire.write(0x6B);
  Wire.write(0);
  if (Wire.endTransmission() != 0) return false;
  Wire.beginTransmission(GYRO);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission();
  return true;
}

void calibrateGyro() {
  delay(1000);
  long sum = 0;
  for (int i = 0; i < 200; i++) {
    Wire.beginTransmission(GYRO);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom(GYRO, 2, true);
    int16_t gz = Wire.read() << 8 | Wire.read();
    sum += gz;
    delay(5);
  }
  gyroOffset = sum / 200.0;
  angle = 0;
  lastTime = millis();
}

int16_t readGyro() {
  Wire.beginTransmission(GYRO);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(GYRO, 2, true);
  return Wire.read() << 8 | Wire.read();
}

void updateGyro() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;
  int16_t gz = readGyro();
  angle += -((gz - gyroOffset) / 131.0) * dt;
}

void resetAngle() {
  angle = 0;
  lastTime = millis();
}

// ===================== TURNS =====================

void turnRight90() {
  stopMotors(); delay(150);
  resetAngle();
  moveMotors(120, -120);
  while (angle > -85) updateGyro();
  stopMotors(); delay(200);
  resetAngle();
}

void turnLeft90() {
  stopMotors(); delay(150);
  resetAngle();
  moveMotors(-120, 120);
  while (angle < 85) updateGyro();
  stopMotors(); delay(200);
  resetAngle();
}

// ===================== ULTRASONIC =====================

int getDistance() {
  digitalWrite(US_OUT, LOW);
  delayMicroseconds(2);
  digitalWrite(US_OUT, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_OUT, LOW);
  long duration = pulseIn(US_IN, HIGH, 30000);
  if (duration == 0) return 999;
  return duration * 0.034 / 2;
}

// ===================== SERVO =====================

void setServoLogical(int ang) {
  scanServo.write(constrain(ang, -90, 90) + 90);
}

void sweepServo() {
  unsigned long now = millis();
  servoPhase += ((now - lastServoUpdate) / 1000.0) * SERVO_SWEEP_SPEED * TWO_PI;
  lastServoUpdate = now;
  if (servoPhase > TWO_PI) servoPhase -= TWO_PI;
  setServoLogical((int)(sin(servoPhase) * 90));
}

// ===================== DETECTION SWEEP =====================
// Stops at each angle, takes READINGS_PER_ANGLE readings spaced
// READING_DELAY ms apart, returns index of angle with highest delta.
// If that delta is below MOVEMENT_THRESHOLD returns -1 (no moving object)

int detectionSweep() {
  int bestIndex = -1;
  int bestDelta = MOVEMENT_THRESHOLD;

  for (int i = 0; i < SCAN_POINTS; i++) {
    setServoLogical(scanAngles[i]);
    delay(300); // settle time

    int minDist = 999, maxDist = 0;

    for (int r = 0; r < READINGS_PER_ANGLE; r++) {
      int d = getDistance();
      if (d < minDist) minDist = d;
      if (d > maxDist) maxDist = d;
      delay(READING_DELAY);
    }

    int delta = maxDist - minDist;
    scanDistances[i] = minDist; // store closest reading for wall avoidance fallback

    if (delta > bestDelta) {
      bestDelta = delta;
      bestIndex = i;
    }
  }

  return bestIndex; // -1 = no movement found, 0-6 = index of most movement
}

// ===================== WALL SCAN =====================

void scanEnvironment() {
  for (int i = 0; i < SCAN_POINTS; i++) {
    setServoLogical(scanAngles[i]);
    delay(250);
    int d = getDistance();
    scanDistances[i] = d;
    memory[i] = (d < 25) ? min(memory[i] + 8, 50) : max(memory[i] - 2, 0);
  }
  setServoLogical(0);
}

int findBestDirection() {
  int bestIndex = 0, bestScore = -1000;
  for (int i = 0; i < SCAN_POINTS; i++) {
    int score = scanDistances[i] - memory[i] * 3;
    if (score > bestScore) { bestScore = score; bestIndex = i; }
  }
  return scanAngles[bestIndex];
}

// ===================== SETUP =====================

void setup() {
  Serial.begin(9600);
  FastLED.addLeds<NEOPIXEL, PIN_RBGLED>(leds, NUM_LEDS);
  FastLED.clear();
  FastLED.show();

  scanServo.attach(SERVO);

  setupMotors();
  setupGyro();
  calibrateGyro();

  pinMode(US_OUT, OUTPUT);
  pinMode(US_IN, INPUT);
  pinMode(BUTTON, INPUT_PULLUP);

  while (digitalRead(BUTTON) == HIGH);
  delay(200);
}

// ===================== LOOP =====================

void loop() {
  updateGyro();

  // White line = hard stop
  if (analogRead(LEFT) <= L_TH && analogRead(MIDDLE) <= M_TH && analogRead(RIGHT) <= R_TH) {
    stopMotors();
    leds[0] = leds[1] = CRGB::White;
    FastLED.show();
    while (1);
  }

  int dist = getDistance();

  // ---- OBJECT DETECTED - do detection sweep to decide ----
  if (dist < DETECTION_RANGE && !following) {
    stopMotors();

    leds[0] = leds[1] = CRGB::Yellow;
    FastLED.show();

    int movingIndex = detectionSweep();

    if (movingIndex >= 0) {
      // Moving object found - lock onto that angle and follow
      following = true;
      followAngleIndex = movingIndex;

      // Copy prevDistances so follow mode has a baseline to compare against
      for (int i = 0; i < SCAN_POINTS; i++)
        prevDistances[i] = scanDistances[i];

    } else {
      // Nothing moving - treat as wall
      following = false;

      scanEnvironment();
      int bestAngle = findBestDirection();

      moveMotors(-80, -80);
      delay(400);
      stopMotors();
      delay(200);

      if (bestAngle > 20) {
        leds[0] = leds[1] = CRGB::Red; FastLED.show();
        turnRight90(); numerOfRights++; lastTurn = 1;
      } else if (bestAngle < -20) {
        leds[0] = leds[1] = CRGB::Purple; FastLED.show();
        turnLeft90(); numerOfLefts++; lastTurn = -1;
      } else {
        if (lastTurn == 1) {
          leds[0] = leds[1] = CRGB::Yellow; FastLED.show();
          turnLeft90(); lastTurn = -1;
        } else {
          leds[0] = leds[1] = CRGB::Orange; FastLED.show();
          turnRight90(); lastTurn = 1;
        }
      }
      resetAngle();
    }

  // ---- FOLLOW MODE ----
  } else if (following) {
    leds[0] = leds[1] = CRGB::Cyan;
    FastLED.show();

    // Sweep and sample each angle
    sweepServo();
    int currentServoAngle = (int)(sin(servoPhase) * 90);
    for (int i = 0; i < SCAN_POINTS; i++) {
      if (abs(currentServoAngle - scanAngles[i]) <= 2)
        scanDistances[i] = getDistance();
    }

    // Find angle with most change since last sweep = where object is moving
    int bestDelta = MOVEMENT_THRESHOLD;
    int bestIndex = -1;
    for (int i = 0; i < SCAN_POINTS; i++) {
      int delta = abs(scanDistances[i] - prevDistances[i]);
      if (delta > bestDelta) {
        bestDelta = delta;
        bestIndex = i;
      }
      prevDistances[i] = scanDistances[i];
    }

    // If we still see movement update follow angle, otherwise keep last known
    if (bestIndex >= 0) followAngleIndex = bestIndex;

    // If no movement seen at all across entire sweep exit follow mode
    if (bestIndex == -1 && dist > DETECTION_RANGE) {
      following = false;
      return;
    }

    int cd = scanDistances[followAngleIndex];

    if (cd < FOLLOW_STOP_DIST) {
      stopMotors();
      return;
    }

    float steer   = scanAngles[followAngleIndex] / 90.0;
    int baseSpeed = map(cd, FOLLOW_STOP_DIST, DETECTION_RANGE, 50, 120);
    int steerAmt  = (int)(steer * 70);

    // Hard turn if object is far to the side
    if (abs(scanAngles[followAngleIndex]) >= 60) {
      scanAngles[followAngleIndex] > 0 ? turnRight90() : turnLeft90();
      resetAngle();
    } else {
      moveMotors(constrain(baseSpeed + steerAmt, 0, 255),
                 constrain(baseSpeed - steerAmt, 0, 255));
    }

  // ---- ROAM MODE ----
  } else {
    if (numerOfRights > 5) { turnLeft90(); numerOfRights = 0; }
    if (numerOfLefts  > 5) { turnRight90(); numerOfLefts = 0; }

    leds[0] = leds[1] = CRGB::Green;
    FastLED.show();
    int speed = constrain(map(dist, 10, 80, 60, 150), 60, 150);
    driveStraight(speed);
  }
}
