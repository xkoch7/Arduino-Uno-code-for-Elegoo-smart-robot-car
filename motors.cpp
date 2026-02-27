#include "Robot.h"
//setup for motors and gyro
void setupMotors()
{
  pinMode(PWR_L, OUTPUT);
  pinMode(PWR_R, OUTPUT);
  pinMode(MTR_L, OUTPUT);
  pinMode(MTR_R, OUTPUT);
  pinMode(MTR_ENABLE, OUTPUT);
  digitalWrite(MTR_ENABLE, HIGH);
}

bool setupGyro()
{
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

void calibrateGyro()
{
  delay(1000);
  long sum = 0;

  for (int i = 0; i < 200; i++)
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
  float rate = -((gz - gyroOffset) / 131.0);
  angle += rate * dt;
}

void resetAngle()
{
  angle = 0;
  lastTime = millis();
}
//movement controls using motors and gyro for perfect straight driving and turns
void moveMotors(int leftSpeed, int rightSpeed)
{
  digitalWrite(MTR_L, leftSpeed >= 0 ? HIGH : LOW);
  digitalWrite(MTR_R, rightSpeed >= 0 ? HIGH : LOW);

  analogWrite(PWR_L, constrain(abs(leftSpeed), 0, 255));
  analogWrite(PWR_R, constrain(abs(rightSpeed), 0, 255));
}

void stopMotors()
{
  analogWrite(PWR_L, 0);
  analogWrite(PWR_R, 0);
}

void driveStraight(int speed)
{
  updateGyro();

  float correction = angle * 3.0;
  correction = constrain(correction, -30, 30);

  int left = speed - correction;
  int right = speed + correction;

  moveMotors(left, right);
}

void turnRight90()
{
  stopMotors();
  delay(150);
  resetAngle();

  moveMotors(75, -75);

  while (angle > -85)
  {
    updateGyro();
  }

  stopMotors();
  delay(200);
  resetAngle();
}

void turnLeft90()
{
  stopMotors();
  delay(150);
  resetAngle();

  moveMotors(-75, 75);

  while (angle < 85)
  {
    updateGyro();
  }

  stopMotors();
  delay(200);
  resetAngle();
}
