// basic pin and Led setup all preset for the Elegoo smart robot car V4.0
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

#define L_TH 380
#define M_TH 50
#define R_TH 75
#define BUTTON 2
#define GYRO 0x68
//initialize servo and Leds
CRGB leds[NUM_LEDS];
Servo scanServo;

// functions to setup, control and use gyro
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
  for(int i = 0; i < 200; i++) {
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
  float rate = (gz - gyroOffset) / 131.0;
  angle += rate * dt;
}

void resetAngle() {
  angle = 0;
  targetHeading = 0;
  lastTime = millis();
}

// setup is where you can set the values for the sensors and motors using the pins
void setup() {

}

void loop() {
  // put your main code here, to run repeatedly:

}

