//-----------------------------------------------------------------------------------LIBRARIES----------------------------------------------------------------------------------------------
#include <Adafruit_NeoPixel.h>
#include <QTRSensors.h>
//-----------------------------------------------------------------------------------VARIABLES AND CONSTANTS--------------------------------------------------------------------------------
//NEOPIXEL VARIABLES
const int PIXEL_PIN=4;
const int PIXEL_NUMBER=4;
Adafruit_NeoPixel leds(PIXEL_NUMBER,PIXEL_PIN,NEO_RGB +NEO_KHZ800);

//NEOPIXELS COLORS 
const uint32_t ORANGE=leds.Color(255,165,0);
const uint32_t RED=leds.Color(255,0,0);
const uint32_t YELLOW=leds.Color(234,255,0);
const uint32_t BLUE=leds.Color(0,0,255);


//MOTOR PINS
const int  MOTOR_A1 = 9;//Left wheel backwards
const int MOTOR_A2 = 10;//Left wheel forward
const int MOTOR_B1 = 6;//Right wheel forward
const int MOTOR_B2 = 5;//Right wheel backwards

//PULSE SENSOR
const int SENSOR_R1 = 3;  //left wheel sensor
const int SENSOR_R2 = 2; //right wheel sensor

bool isFinished = false;

int Signal_R1;
int Signal_R2;               


//LINE SENSOR VARIABLES
QTRSensors lineSensors;
const int SENSOR_COUNT = 8;
int sensorValues[SENSOR_COUNT];
int position = 0;

long timeSinceStartup;


// //BUTTON PINS
// const int BUTTON1 = 7;
// const int BUTTON2 = 8;
// const int BUTTON3 = 12;

// //BUTTON STATES
// int buttonState1 = 0;
// int buttonState2 = 0;
// int buttonState3 = 0;

//OBJECT SENSOR
const int echoPin=11;
const int trigPin=8;

//SENSOR VARIABLES
long duration;
int distance;

//GRIPPER
const int GRIPPER_PIN=12;
const int GRIPPER_OPEN_PULSE=1600;
const int GRIPPER_CLOSE_PULSE=971;
const int GRIPPER_PULSE_REPEAT=10;

//---------------------------------------------------------------------------------------FUNCTION HEADERS-----------------------------------------------------------------------------------
//MOVING FUNTIONS
void setupMotorPins();//setup the pin mode for the pins moving the DC motors
void still();//keeps the motors still
void stop();//stop the robot
void moveForward(int powerA = 255,int powerB = 255);//move the robot forward
void moveBackward(int powerA = 255,int powerB = 255);//move the robot backwards
void turnBeforeObject(int powerA = 255, int powerB = 255);//turn if sees an obstacle

// ROBOT CONTROL FUNCTIONS
void setupButtons();
void button_control();

//LINE SENSORS
void setupLineSensors();
void calibration();
bool isBlack(int value);
void showSensorValues();
//LINE FOLLOWER FUNCTIONS
bool isBlack(int value); //checks if a sensor value represents black or white
bool lineAhead(); // checks if the line is situated straight ahead of the robot
bool lineRight(); // checks if the line begins to curve right
bool lineLeft(); // checks if the line begins to curve 
bool almostBlack(); // checks if there is a crossroad ahead
bool allBlack(); //checks if there is a perpendicular line on the track simbolizing a crossroads or a beginning line 
void lineFollow(); //contains the line follower algorithm
void beginRace();

// //DISTANCE SENSOR FUNCTIONS
void readDistance();//Reads distance

//GRIPPER
void gripperServo(int pulse);
void openGripper();
void closeGripper();
//---------------------------------------------------------------------------------FUNCTION DECLARATIONS------------------------------------------------------------------------------------
// SETUP AND LOOP FUNTIONS
void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  timeSinceStartup = millis();
  openGripper();
  leds.begin();
  Serial.begin(9600);
  setupMotorPins();
  // setupButtons();
  setupLineSensors();
  calibration();

  still();
  Serial.println("Beginning Race");
  while(!allBlack())
  {
    position=lineSensors.readLineBlack(sensorValues);
    showSensorValues();
    beginRace();
  }
  Serial.println("Picking up object");
  still();
  closeGripper();
  Serial.println("Picked up the object");
  still();
  moveForward(0,170);
  delay(750);
  if (allBlack() || almostBlack())
  {
    while (allBlack() || almostBlack())
      position=lineSensors.readLineBlack(sensorValues);
    still();
  }
  still();
  moveForward(180,180);
  delay(200);
  still();
}

void loop() {

//  button_control();
  leds.show();
  position=lineSensors.readLineBlack(sensorValues);
  showSensorValues();
  lineFollow();
  readDistance();

  if (distance < 16 && millis() > timeSinceStartup + 5000)
  {
    avoidObject();
  }
    
  if(isFinished==true)
  {
    still();
    moveBackward(200,200);
    delay(1000);
    still();
    // moveForward(180,0);
    stop();
    leds.clear();
    leds.fill(RED,0,1);
    leds.fill(BLUE,2,2);
    leds.fill(ORANGE,1,1);
    delay(1000);
    leds.clear();
    delay(1000);
    stop();
    delay(10000);
  }

}

//LINE SENSOR FUNCTIONS
void setupLineSensors()
{
  lineSensors.setTypeAnalog();
  lineSensors.setSensorPins((const uint8_t[]){A0,A1,A2,A3,A4,A5,A6,A7},SENSOR_COUNT); 
} 

//MOTOR FUNCTIONS
void stop()
{
  leds.clear();
  leds.fill(RED,2,2);
  leds.show();
  moveForward(0, 0);
  moveBackward(0, 0);
}

void still()
{
  int motor_pins[4]={MOTOR_A1,MOTOR_A2,MOTOR_B1,MOTOR_B2};
  leds.clear();
  for (int i=0;i<4;i++)
    {      
      analogWrite(motor_pins[i],0);
    }
}

void setupMotorPins()
{
  pinMode(MOTOR_A1,OUTPUT);
  pinMode(MOTOR_A2,OUTPUT);
  pinMode(MOTOR_B1,OUTPUT);
  pinMode(MOTOR_B2,OUTPUT);
  pinMode(SENSOR_R1,INPUT);
  pinMode(SENSOR_R2,INPUT);
}

void moveForward(int powerA=255,int powerB=255)
{
  leds.clear();
  leds.fill(BLUE,2,2);
  leds.show(); 
  analogWrite(MOTOR_A2,powerA);
  analogWrite(MOTOR_B1,powerB);
}

void moveBackward(int powerA=255,int powerB=255)
{
  leds.clear();
  leds.fill(RED,0,2);
  leds.show();
  analogWrite(MOTOR_A1,powerA);
  analogWrite(MOTOR_B2,powerB);
}

void calibration()
{
  //begin the calibration process
   for(int i = 0; i < 2;i++)
   {
        moveForward(165,185);
        for(uint8_t i = 0; i < 25 ; i++)
        {
            lineSensors.calibrate();
            Serial.print("Calibration: ");
            Serial.println(i);
        }
        still();
        delay(100);
        moveBackward(155,175);
        for(uint8_t i = 0; i < 25 ; i++)
        {
            lineSensors.calibrate();
            Serial.print("Calibration: ");
            Serial.println(i);
        }
        Serial.println("Calibration done");

    }
    //end of the calibration
    still();
    moveForward(180,205);
    delay(1500);
    still();
}
void showSensorValues()
{
  for (int i=0;i<SENSOR_COUNT;i++)
    {
      Serial.print(isBlack(sensorValues[i]));
      Serial.print(" ");
    }
   Serial.println();
}
//LINE FOLLOWER LOGIC FUNCTIONS
bool isBlack(int value)
{
  return value>700;
}

bool lineAhead()
{
  if (!isBlack(sensorValues[3]) || !isBlack(sensorValues[4]))
    return false;

  for (int i=0;i<SENSOR_COUNT;i++)
  {
    if (i==3 || i==4)
      continue;
    if (isBlack(sensorValues[i]))
      return false;
  }
  return true;
}

bool lineRight()
{
  for (int i=SENSOR_COUNT/2+1;i<SENSOR_COUNT;i++)
  {
    if (isBlack(sensorValues[i]))
      return false;
  }
  return true;
}

bool lineLeft()
{
  for (int i=0;i<SENSOR_COUNT/2-1;i++)
  {
    if (isBlack(sensorValues[i]))
      return false;
  }
  return true;
}

bool almostBlack()
{
  int k=0;
  for (int i=0;i<SENSOR_COUNT/2-1;i++)
    if (isBlack(sensorValues[i]))
      k=1;
  for (int i=SENSOR_COUNT/2+1;i<SENSOR_COUNT;i++)
    if (isBlack(sensorValues[i]))
      k=2;
  if (k==2)
    return true;
 return false;
}

bool allBlack()
{
  for (int i=0;i<SENSOR_COUNT;i++)
    if (!isBlack(sensorValues[i]))
        return false;
  return true;  
}

bool allWhite()
{
  for (int i=0;i<SENSOR_COUNT;i++)
    if (isBlack(sensorValues[i]))
        return false;
  return true;
}

void lineFollow()
{
  if (allBlack())
  {
    still();
    moveForward(180,180);
    delay(150);
    still();
    position=lineSensors.readLineBlack(sensorValues);
    //if the line is still black then this is the end of the race and the object will be dropped
    if (allBlack())
    {
      moveBackward(180, 180);
      delay(450);
      stop();
      delay(500);
      openGripper();
      isFinished=true;
      stop();
    }
  
  }
  else if (allWhite())
    {
      moveForward(200, 200);
    }
  else if (lineAhead())
  {
    moveForward(200,200);
  }
  else  if (lineRight())
  {
    // still();
    moveForward(180,15);
  }
 else if (lineLeft())
  {

    moveForward(15,180);
  }
 else if (almostBlack())
 {
  moveForward(200,200);
  delay(200);
 }
  else
  still();
}

void beginRace()
{
  if (lineAhead())
  {
    moveForward(200,200);
  }
  else  if (lineRight())
  {
    still();
    moveForward(200,0);
  }
 else if (lineLeft())
  {
    still();
    moveForward(0,200);
  }
 else if (allWhite())
 {
  still();
 }
}

void readDistance()
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  //Sets trigPin on High state for 10 microSec
  digitalWrite(trigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  //Reads echoPin, returns the wave travel time in miceoSec
  duration=pulseIn(echoPin,HIGH);
  //Calculate distance
  distance=duration*0.034/2;
}

void avoidObject()
{
  int turn=420;
  still();
  leds.clear();
  leds.fill(ORANGE,2,2);
  leds.show();
  delay(1000);
  //get off the track
  moveForward(200,0);
  delay(turn);
  still();
  moveForward(200,200);
  delay(1100);
  still();
  moveForward(0,200);
  delay(3*turn);
  still();
  moveForward(200,200);
  while (allWhite() || almostBlack())
    position=lineSensors.readLineBlack(sensorValues);
  still();  
  stop();
  delay(300);
  moveForward(200, 0);
  delay(300);
}

void gripperServo(int pulse)
{
    for(int i = 0; i < GRIPPER_PULSE_REPEAT;i++)
    {
        digitalWrite(GRIPPER_PIN,HIGH);
        delayMicroseconds(pulse);
        digitalWrite(GRIPPER_PIN,LOW);
        delay(20);
    }
}
void openGripper()
{
    gripperServo(GRIPPER_OPEN_PULSE);
}
void closeGripper()
{
  gripperServo(GRIPPER_CLOSE_PULSE);
}



