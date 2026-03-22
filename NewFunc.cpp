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
#define OPEN_THRESHOLD 10
#define OPEN_MIN 30
#define FOLLOW_STOP 12

CRGB leds[NUM_LEDS];
Servo scanServo;

int lastTurn = 1;
int numerOfLefts = 0;
int numerOfRights = 0;

float angle = 0;
float gyroOffset = 0;
unsigned long lastTime = 0;

int scanAngles[SCAN_POINTS]   = {-90, -60, -30, 0, 30, 60, 90};
int scanDistances[SCAN_POINTS] = {999, 999, 999, 999, 999, 999, 999};
int memory[SCAN_POINTS]        = {0};

bool seeking = false;
int seekIndex = 3;

void setupMotors()
{
  pinMode(PWR_L, OUTPUT);
  pinMode(PWR_R, OUTPUT);
  pinMode(MTR_L, OUTPUT);
  pinMode(MTR_R, OUTPUT);
  pinMode(MTR_ENABLE, OUTPUT);
  digitalWrite(MTR_ENABLE, HIGH);
}

void stopMotors()
{
  analogWrite(PWR_L, 0);
  analogWrite(PWR_R, 0);
  digitalWrite(MTR_L, LOW);
  digitalWrite(MTR_R, LOW);
}

void moveMotors(int leftSpeed, int rightSpeed)
{
  digitalWrite(MTR_L, leftSpeed  >= 0 ? HIGH : LOW);
  digitalWrite(MTR_R, rightSpeed >= 0 ? HIGH : LOW);
  analogWrite(PWR_L, constrain(abs(leftSpeed),  0, 255));
  analogWrite(PWR_R, constrain(abs(rightSpeed), 0, 255));
}

void driveStraight(int speed)
{
  float correction = constrain(angle * 12.0, -70, 70);
  int left  = constrain(speed - correction + 5, 0, 255);
  int right = constrain(speed + correction - 5, 0, 255);
  moveMotors(left, right);
  angle *= 0.97;
}

bool setupGyro()
{
  Wire.begin();
  Wire.beginTransmission(GYRO);
  Wire.write(0x6B);
  Wire.write(0);
  if(Wire.endTransmission() != 0) return false;
  Wire.beginTransmission(GYRO);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission();
  return true;
}

void calibrateGyro()
{
  delay(1000);
  long sum = 0;
  for(int i = 0; i < 200; i++)
  {
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

int16_t readGyro()
{
  Wire.beginTransmission(GYRO);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(GYRO, 2, true);
  return Wire.read() << 8 | Wire.read();
}

void updateGyro()
{
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;
  int16_t gz = readGyro();
  angle += -((gz - gyroOffset) / 131.0) * dt;
}

void resetAngle()
{
  angle = 0;
  lastTime = millis();
}

void turnRight90()
{
  stopMotors();
  delay(150);
  resetAngle();
  moveMotors(120, -120);
  while(angle > -85) updateGyro();
  stopMotors();
  delay(200);
  resetAngle();
}

void turnLeft90()
{
  stopMotors();
  delay(150);
  resetAngle();
  moveMotors(-120, 120);
  while(angle < 85) updateGyro();
  stopMotors();
  delay(200);
  resetAngle();
}

int getDistance()
{
  digitalWrite(US_OUT, LOW);
  delayMicroseconds(2);
  digitalWrite(US_OUT, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_OUT, LOW);
  long duration = pulseIn(US_IN, HIGH, 30000);
  if(duration == 0) return 999;
  return duration * 0.034 / 2;
}

void setServoLogical(int ang)
{
  scanServo.write(constrain(ang, -90, 90) + 90);
}

void scanEnvironment()
{
  for(int i = 0; i < SCAN_POINTS; i++)
  {
    setServoLogical(scanAngles[i]);
    delay(250);
    int d = getDistance();
    scanDistances[i] = d;
    if(d < 25)
      memory[i] = min(memory[i] + 8, 50);
    else
      memory[i] = max(memory[i] - 2, 0);
  }
  setServoLogical(0);
}

int findBestDirection()
{
  int bestIndex = 0;
  int bestScore = -1000;
  for(int i = 0; i < SCAN_POINTS; i++)
  {
    int score = scanDistances[i] - memory[i] * 3;
    if(score > bestScore)
    {
      bestScore = score;
      bestIndex = i;
    }
  }
  return scanAngles[bestIndex];
}

int findOpenSpace()
{
  int furthestIndex = -1;
  int furthestDist  = OPEN_MIN;

  for(int i = 1; i < SCAN_POINTS - 1; i++)
  {
    if(scanDistances[i] > furthestDist && scanDistances[i] < 999)
    {
      furthestDist  = scanDistances[i];
      furthestIndex = i;
    }
  }

  if(furthestIndex == -1) return -1;

  long sum = 0;
  int count = 0;
  for(int i = 0; i < SCAN_POINTS; i++)
  {
    if(i == furthestIndex) continue;
    if(scanDistances[i] < 999) { sum += scanDistances[i]; count++; }
  }

  if(count == 0) return -1;

  int avg = sum / count;
  if(furthestDist - avg >= OPEN_THRESHOLD) return furthestIndex;

  return -1;
}

void setup()
{
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

  while(digitalRead(BUTTON) == HIGH);
  delay(200);
}

void loop()
{
  updateGyro();

  int L = analogRead(LEFT);
  int M = analogRead(MIDDLE);
  int R = analogRead(RIGHT);

  if(L <= L_TH && M <= M_TH && R <= R_TH)
  {
    stopMotors();
    leds[0] = leds[1] = CRGB::White;
    FastLED.show();
    while(1);
  }

  int distance = getDistance();

  if(numerOfRights > 5)
  {
    turnLeft90();
    numerOfRights = 0;
  }

  if(numerOfLefts > 5)
  {
    turnRight90();
    numerOfLefts = 0;
  }

  if(distance < DETECTION_RANGE)
  {
    leds[0] = leds[1] = CRGB::Blue;
    FastLED.show();

    stopMotors();
    delay(200);

    scanEnvironment();

    int openIndex = findOpenSpace();

    if(openIndex >= 0)
    {
      seeking = true;
      seekIndex = openIndex;

      leds[0] = leds[1] = CRGB::Cyan;
      FastLED.show();

      int targetAngle = scanAngles[seekIndex];

      if(targetAngle > 0)
      {
        stopMotors();
        delay(150);
        resetAngle();
        moveMotors(120, -120);
        while(angle > -targetAngle) updateGyro();
        stopMotors();
        delay(200);
        resetAngle();
      }
      else if(targetAngle < 0)
      {
        stopMotors();
        delay(150);
        resetAngle();
        moveMotors(-120, 120);
        while(angle < -targetAngle) updateGyro();
        stopMotors();
        delay(200);
        resetAngle();
      }
    }
    else
    {
      seeking = false;

      int bestAngle = findBestDirection();

      moveMotors(-80, -80);
      delay(400);
      stopMotors();
      delay(200);

      if(bestAngle > 20)
      {
        leds[0] = leds[1] = CRGB::Red;
        FastLED.show();
        turnRight90();
        numerOfRights++;
        lastTurn = 1;
      }
      else if(bestAngle < -20)
      {
        leds[0] = leds[1] = CRGB::Purple;
        FastLED.show();
        turnLeft90();
        numerOfLefts++;
        lastTurn = -1;
      }
      else
      {
        if(lastTurn == 1)
        {
          leds[0] = leds[1] = CRGB::Yellow;
          FastLED.show();
          turnLeft90();
          lastTurn = -1;
        }
        else
        {
          leds[0] = leds[1] = CRGB::Orange;
          FastLED.show();
          turnRight90();
          lastTurn = 1;
        }
      }
      resetAngle();
    }
  }
  else
  {
    seeking = false;
    leds[0] = leds[1] = CRGB::Green;
    FastLED.show();
    int speed = map(distance, 10, 80, 60, 150);
    speed = constrain(speed, 60, 150);
    driveStraight(speed);
  }
}
