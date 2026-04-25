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
#define L_TH 500
#define M_TH 500
#define R_TH 500

#define GYRO 0x68
#define SCAN_POINTS 7
#define MAX_TURNS 4

enum Mode { MAZE, LINE };
Mode currentMode = MAZE;

int modeSwitchCount = 0;

CRGB leds[NUM_LEDS];
Servo scanServo;

int numerOfLefts = 0;
int numerOfRights = 0;
int ultrasonicFailCount = 0;

bool blockRight = false;
bool blockLeft = false;

int lastTurn = 1;

float angle = 0;
float gyroOffset = 0;
unsigned long lastTime = 0;

int scanAngles[SCAN_POINTS] = {-90,-60,-30,0,30,60,90};
int scanDistances[SCAN_POINTS];
int memory[SCAN_POINTS] = {0};


void stopMotors() {
  analogWrite(PWR_L, 0);
  analogWrite(PWR_R, 0);
}

void moveMotors(int leftSpeed, int rightSpeed)
{
  digitalWrite(MTR_L, leftSpeed >= 0 ? HIGH : LOW);
  digitalWrite(MTR_R, rightSpeed >= 0 ? HIGH : LOW);

  analogWrite(PWR_L, constrain(abs(leftSpeed), 0, 255));
  analogWrite(PWR_R, constrain(abs(rightSpeed), 0, 255));
}

void setupMotors()
{
  pinMode(PWR_L, OUTPUT);
  pinMode(PWR_R, OUTPUT);
  pinMode(MTR_L, OUTPUT);
  pinMode(MTR_R, OUTPUT);
  pinMode(MTR_ENABLE, OUTPUT);
  digitalWrite(MTR_ENABLE, HIGH);
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

void rightWheelForward(int time, int speed) {
  digitalWrite(MTR_L, HIGH);
  analogWrite(PWR_R, speed);
  delay(time);
  analogWrite(PWR_R, 0);
}

void leftWheelForward(int time, int speed) {
  digitalWrite(MTR_R, HIGH);
  analogWrite(PWR_L, speed);
  delay(time);
  analogWrite(PWR_L, 0);
}

void stopWheels() {
  analogWrite(PWR_R, 0);
  analogWrite(PWR_L, 0);
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


void driveStraight(int speed)
{
  updateGyro();

  float correction = angle * 12.0;
  correction = constrain(correction, -70, 70);

  int bias = 5;
  int left = speed - correction + bias;
  int right = speed + correction - bias;

  left = constrain(left, 0, 255);
  right = constrain(right, 0, 255);

  moveMotors(left, right);
  angle *= 0.97;
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



void setServoLogical(int angle)
{
  angle = constrain(angle, -90, 90);
  scanServo.write(angle + 90);
}
//scanning invirment to determine whitch direction to go
void scanEnvironment()
{
  for(int i = 0; i < SCAN_POINTS; i++)
  {
    setServoLogical(scanAngles[i]);
    delay(200);
    scanDistances[i] = getDistance();
  }
  setServoLogical(0);
}
void turnAround() {
  // Spin in place (same direction as your normal right turn)
  digitalWrite(PWR_L, HIGH);
  digitalWrite(PWR_L, LOW);
  digitalWrite(PWR_R, LOW);
  digitalWrite(PWR_R, HIGH);

  delay(800);
  //delay after turning around to now overload data

  stopMotors();
}


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

  pinMode(US_OUT, OUTPUT);
  pinMode(US_IN, INPUT);
  pinMode(BUTTON, INPUT_PULLUP);

  while(digitalRead(BUTTON) == HIGH);
  delay(200);
}

int finishCounter = 0;
bool doCelebrate = false;

void loop()
{
//constant values alawys updated in loop
  int distance = getDistance();
  updateGyro();
  int L = analogRead(LEFT);
  int M = analogRead(MIDDLE);
  int R = analogRead(RIGHT);

  //if enters maze solving mode it will increase the mode switch
  if(currentMode == MAZE)
  {
    modeSwitchCount++; 
    // Switch to line following if all sensors see white
    if (L < L_TH && M < M_TH && R < R_TH)
    {
      stopMotors();
      delay(300); // Settle before starting line follow
      currentMode = LINE;
      return; 
    }

    if(distance < 5)
    {
      stopMotors();
      delay(400);
      scanEnvironment(); 
      resetAngle();

      int farLeftDist = (scanDistances[0] + scanDistances[1]) / 2;
      int farRightDist = (scanDistances[5] + scanDistances[6]) / 2;
      
      if ((farLeftDist >= 100 && farRightDist >= 100) || (farLeftDist >= farRightDist)) 
      {
        turnLeft90();
      }
      else 
      {
        turnRight90();
      }
    }
    else
    {
      driveStraight(60);
    }
  }
  // line following
  else if(currentMode == LINE)
  {
    if (L > L_TH && M > M_TH && R > R_TH)
    {
      stopMotors();
      delay(400);
      if (analogRead(LEFT) > L_TH && analogRead(MIDDLE) > M_TH && analogRead(RIGHT) > R_TH) {
        modeSwitchCount++; 
        currentMode = MAZE;
        return;
      }
    }
    
    
    if (L < L_TH && R < R_TH) 
    {
      
      driveStraight(45); 
    }
    else if (L > L_TH && R < R_TH) 
    {
      
      moveMotors(-70, 90); 
    }
    else if (R > R_TH && L < L_TH) 
    {
      moveMotors(90, -70); 
    }
    else 
    {
      moveMotors(40, 40);
    }
  }
  //setting celebrate condition
  if(modeSwitchCount >= 3 && !doCelebrate)
  {
    doCelebrate = true;
  }
  //super cool celebration
  if(doCelebrate)
  {
    stopMotors();
    for(int i = 0; i < 8; i++) {
      moveMotors(180, -180);
      delay(250);
      moveMotors(-180, 180);
      delay(250);
    }
    stopMotors();
    while(true); 
  }
}
