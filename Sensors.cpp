#include "Robot.h"
#include "motors.cpp"
//function to use the ultra sonic sensor to get distance
int getDistance() {
  digitalWrite(US_OUT, LOW);
  delayMicroseconds(2);
  digitalWrite(US_OUT, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_OUT, LOW);

  long duration = pulseIn(US_IN, HIGH, 30000);
  if(duration == 0) return 999;

  return duration * 0.034 / 2;
}
//function that uses the light sensors to detect a white line (values can be changed to detect different coloured lines)
bool detectLine(){
  if ((analogRead(Left) < 380) && (analogRead(Middle) < 50) && (analogRead(Right) < 75)){
    return true;
  }
  return false;
}
void followLine() {
  int leftVal = analogRead(Left);
  int midVal  = analogRead(Middle);
  int rightVal = analogRead(Right);

  if (midVal < 50) {
    driveStraight(100);
  } 
  else if (leftVal < 380) {
    resetAngle();
    moveMotors(-80, 80);
  } 
  else if (rightVal < 75) {
    resetAngle();
    moveMotors(80, -80);
  } 
  else {
    stopMotors();
  }
}
void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
