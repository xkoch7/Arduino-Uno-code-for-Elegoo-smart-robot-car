#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <Wire.h>

extern float angle;
extern float gyroOffset;
extern float lastTime;
extern int16_t gyroZ;

void setupMotors();
void moveMotors(int leftSpeed, int rightSpeed);
void stopMotors();
void driveStraight(int speed);
void turnRight90();
void turnLeft90();
void resetAngle();
void updateGyro();

#endif
