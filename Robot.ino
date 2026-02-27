#include <Wire.h>
#include <FastLED.h>
#include <Servo.h>
#include "Robot.h"

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

#define GYRO 0x68

CRGB leds[NUM_LEDS];
Servo scanServo;

float angle = 0;
float gyroOffset = 0;
unsigned long lastTime = 0;

void setup()
{
  Serial.begin(9600);

  FastLED.addLeds<NEOPIXEL, PIN_RBGLED>(leds, NUM_LEDS);
  FastLED.clear();
  FastLED.show();

  scanServo.attach(SERVO);
  scanServo.write(90);

  setupMotors();
  setupGyro();
  calibrateGyro();
}

void loop()
{

}
