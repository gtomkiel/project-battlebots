//>--[Libraries]

#include <Adafruit_NeoPixel.h>
#include <QTRSensors.h> 

//>--[Pins]

#define PIN        7
#define NUMPIXELS  4

#define trigPin 13
#define echoPin 4 

const int grp = 9;

const int motorLeftFWD = 5;
const int motorLeftBWD = 6;
const int motorRightFWD = 11;
const int motorRightBWD = 12;

const int R1 = 3;
const int R2 = 2;

const int sensorOne = A0;
const int sensorTwo = A1;
const int sensorThree = A2;
const int sensorFour = A3;
const int sensorFive = A4;
const int sensorSix = A5;
const int sensorSeven = A6;
const int sensorEight = A7;

//>--[Variables]

volatile int countL = 0;
volatile int countR = 0;

volatile long duration;
volatile int distance;

bool wait = true;
bool set = false;
bool solved = false;
bool victory = false;

QTRSensors qtr;
const uint8_t SensorCount = 8;

uint16_t sensorValues[SensorCount];

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(9600);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A7, A6, A5, A4, A3, A2, A1, A0}, SensorCount);

  pixels.begin();

  pixels.clear();
  pixels.show();

  pinMode(grp, OUTPUT);

  pinMode(sensorOne, INPUT);
  pinMode(sensorTwo, INPUT);
  pinMode(sensorThree, INPUT);
  pinMode(sensorFour, INPUT);
  pinMode(sensorFive, INPUT);
  pinMode(sensorSix, INPUT);
  pinMode(sensorSeven, INPUT);
  pinMode(sensorEight, INPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(motorLeftFWD, OUTPUT);
  pinMode(motorRightFWD, OUTPUT);
  pinMode(motorLeftBWD, OUTPUT);
  pinMode(motorRightBWD, OUTPUT);

  pinMode(R1, INPUT);
  pinMode(R2, INPUT);

  attachInterrupt(digitalPinToInterrupt(3), rotationL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(2), rotationR, CHANGE);
}

void loop() {  
  qtr.read(sensorValues);

  if(wait)
  {
    detectObject();
  }
  else if (!set)
  {
    start();
  }
  else if (!solved)
  {
    maze();
  }
  else if (!victory)
  {
    success();
  }
}

//>--[Functions]

/* Interrupt functions for counting rotation of the motor */
void rotationL()
{
  countL++;
}

void rotationR()
{
  countR++;
}

/* Function handling logic for initiating the line maze and picking up the flag */
void start()
{
  delay(3000);

  int lines = 0;

  forwardSlow();

  while (true)
  {
    qtr.read(sensorValues);

    forwardSlow();

    if(sensorValues[0] > 700 && sensorValues[1] > 700 && sensorValues[2] > 700 && sensorValues[3] > 700 && sensorValues[4] > 700 && sensorValues[5] > 700 && sensorValues[6] > 700 && sensorValues[7] > 700)
    {
        lines += 1;
        delay(200);
    }
    if (sensorValues[0] < 700 && sensorValues[1] < 700 && sensorValues[2] < 700 && sensorValues[3] > 700 && sensorValues[4] > 700 && sensorValues[5] < 700 && sensorValues[6] < 700 && sensorValues[7] < 700)
    {
        forward();
    } 
    else if (sensorValues[0] < 700 && sensorValues[1] < 700 && sensorValues[2] < 700 && sensorValues[5] > 700 || sensorValues[6] > 700 || sensorValues[7] > 700)
    {
        adjustRight();
    }
    else if (sensorValues[0] > 700 || sensorValues[1] > 700 || sensorValues[2] > 700 && sensorValues[5] < 700 && sensorValues[6] < 700 && sensorValues[7] < 700)
    {
        adjustLeft();
    }

    if (lines > 4)
    {
      stop();
      delay(200);
      analogWrite(grp, 40);
      delay(50);
      analogWrite(grp, 0);
      delay(1000);
      leftTurn(33);
      break;
    }
  }
  set = true;
}

/* Function handling logic for solving the line maze */
void maze()
{
   if (sensorValues[4] > 700 && sensorValues[5] > 700 && sensorValues[6] > 700 && sensorValues[7] > 700) // &&
    {
      forward();
      delay(100);
      stop();

      while(true)
      {
        /* There might be slight issues if two possible ways   */
        /* are left and right turn. If the robot comes to this */
        /* section on some weird angle there is a changce that */
        /* it will interpret it as a finish line of the maze.  */

        qtr.read(sensorValues);

        if (sensorValues[0] > 700 && sensorValues[1] > 700 && sensorValues[2] > 700 && sensorValues[3] > 700 && sensorValues[4] > 700 && sensorValues[5] > 700 && sensorValues[6] > 700 && sensorValues[7] > 700)
        {
          stop();
          delay(250);
          analogWrite(grp, 200);
          solved = true;
          break;
        }
        else
        {
          rightTurn(42);
          break;
        }
      }
    }
    else if (sensorValues[3] > 700 && sensorValues[4] > 700)
    {
      forward();
    } 
    else if (sensorValues[5] > 700 || sensorValues[6] > 700 || sensorValues[7] > 700)
    {
      adjustRight();
    }
    else if (sensorValues[0] > 700 || sensorValues[1] > 700 || sensorValues[2] > 700)
    {
      adjustLeft();
    }
    else if (sensorValues[0] < 700 && sensorValues[1] < 700 && sensorValues[2] < 700 && sensorValues[3] < 700 && sensorValues[4] < 700 && sensorValues[5] < 700 && sensorValues[6] < 700 && sensorValues[7] < 700)
    {
      /* Left turns are treated as a part of sequence for dead end spots  */
      /* If there is no option to go either forward or right it will move */
      /* forward off the line and make a 90 degree turn left. After it is */
      /* done the robot checks if there is line, if not then function     */
      /* leftSpin() is called which forces robot to turn left untill it   */
      /* finds the line.                                                  */

      forward();
      delay(100);
      leftTurn2(26);

      while(true)
      {
        qtr.read(sensorValues);

        if (sensorValues[0] > 700 || sensorValues[1] > 700 || sensorValues[2] > 700 || sensorValues[3] > 700 || sensorValues[4] > 700 || sensorValues[5] > 700 || sensorValues[6] > 700 || sensorValues[7] > 700)
        {
          break;
        }

        if (sensorValues[0] < 700 && sensorValues[1] < 700 && sensorValues[2] < 700 && sensorValues[3] < 700 && sensorValues[4] < 700 && sensorValues[5] < 700 && sensorValues[6] < 700 && sensorValues[7] < 700)
        {
          leftSpin();
        }
      }
    }
}

/* Little victory dance */
void success()
{
  backward();
  delay(500);
  leftTurn2(135);
  stop();
  victory = true;
}

void forward()
{
    analogWrite(motorLeftFWD, 250);
    analogWrite(motorRightFWD, 250);
    analogWrite(motorLeftBWD, 0);
    analogWrite(motorRightBWD, 0);
}

void forwardSlow()
{
  analogWrite(motorLeftFWD, 140);
  analogWrite(motorRightFWD, 130);
  analogWrite(motorLeftBWD, 0);
  analogWrite(motorRightBWD, 0);
}

void backward()
{
  analogWrite(motorLeftFWD, 0);
  analogWrite(motorRightFWD, 0);
  analogWrite(motorLeftBWD, 250);
  analogWrite(motorRightBWD, 250);
}

void adjustLeft()
{
  analogWrite(motorLeftFWD, 130);
  analogWrite(motorRightFWD, 250);
  analogWrite(motorLeftBWD, 0);
  analogWrite(motorRightBWD, 0);
}

void adjustRight()
{
  analogWrite(motorLeftFWD, 250);
  analogWrite(motorRightFWD, 130);
  analogWrite(motorLeftBWD, 0);
  analogWrite(motorRightBWD, 0);
}

void stop()
{
    analogWrite(motorLeftFWD, 0);
    analogWrite(motorRightFWD, 0);
    analogWrite(motorLeftBWD, 0);
    analogWrite(motorRightBWD, 0);
}

/* Only version of right turn, it is using only one motor*/
void rightTurn(int c)
{
  bool rotate = true;
  int cycles = c;

  countL = 0;
    
    while (rotate == true)
    {
      if(countL < cycles)
      {
        analogWrite(motorLeftFWD, 200);
        analogWrite(motorRightFWD, 0);
        analogWrite(motorLeftBWD, 0);
        analogWrite(motorRightBWD, 0);
      }
      else if(countL > cycles)
      {
        rotate = false;
      }
    }
}

/* First version of left turn, this one uses only one motor*/
void leftTurn(int c)
{
  bool rotate = true;
  int cycles = c;

  countR = 0;
    
    while (rotate == true)
    {
      if(countR < cycles)
      {
        analogWrite(motorLeftFWD, 0);
        analogWrite(motorRightFWD, 200);
        analogWrite(motorLeftBWD, 0);
        analogWrite(motorRightBWD, 0);
      }
      else if(countR > cycles)
      {
        rotate = false;
      }
    }
}

/* Second version of left turn, this one uses both motors */
void leftTurn2(int c)
{
  bool rotate = true;
  int cycles = c;

  countR = 0;
    
    while (rotate == true)
    {
      if(countR < cycles)
      {
        analogWrite(motorLeftFWD, 0);
        analogWrite(motorRightFWD, 150);
        analogWrite(motorLeftBWD, 150);
        analogWrite(motorRightBWD, 0);
      }
      else if(countR > cycles)
      {
        rotate = false;
      }
    }
}

void leftSpin()
{
  analogWrite(motorLeftFWD, 0);
  analogWrite(motorRightFWD, 130);
  analogWrite(motorLeftBWD, 130);
  analogWrite(motorRightBWD, 0);
}

/* Function setting blinking LEDs when the robot is waiting for the start (2 modes available) */
/* did not test the lights yet but they might be slightly broken or working inconsistently */
void blink(int mode) 
{
  switch(mode)
  {
    case 1:
      for(int i=0; i<NUMPIXELS; i++)
      {
        delay(5);

        pixels.clear();
        pixels.show();

        delay(2000);

        for(int pin=0; pin<NUMPIXELS; pin++)
        {
          pixels.setPixelColor(pin, pixels.Color(0, 150, 0));
        }

        pixels.show();
      }
      break;

    case 2:
      for(int i=0; i<NUMPIXELS; i++)
      {
        delay(5);
        pixels.clear();
        pixels.show();

        delay(500);
        pixels.setPixelColor(i, pixels.Color(0, 150, 0));
        pixels.setPixelColor(i+=1, pixels.Color(0, 150, 0));
        pixels.show();
      }
      break;

    default:
      break;
  }
}

/* Function returning distance to nearby object (required for the function object() to work)*/
int getDistance() 
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;

  return distance;
}

void detectObject()
{
  int distance = getDistance();

  if(distance < 25)
  {
    wait = false;
  } 
}
