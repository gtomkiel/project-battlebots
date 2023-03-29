//-----------------------------------------------------------------------------------LIBRARIES----------------------------------------------------------------------------------------------
#include <Adafruit_NeoPixel.h>
#include <QTRSensors.h>
//-----------------------------------------------------------------------------------VARIABLES AND CONSTANTS--------------------------------------------------------------------------------
//NEOPIXEL VARIABLES
const int PIXEL_PIN=4;
const int PIXEL_NUMBER=4;
Adafruit_NeoPixel leds(PIXEL_NUMBER,PIXEL_PIN,NEO_RGB +NEO_KHZ800);

//NEOPIXELS COLORS 
const uint32_t RED=leds.Color(255,0,0);
const uint32_t YELLOW=leds.Color(234,255,0);
const uint32_t BLUE=leds.Color(0,0,255);

//MOTOR PINS
const int  MOTOR_A1 = 9;//Left wheel backwards
const int MOTOR_A2 = 10;//Left wheel forward
const int MOTOR_B1 = 6;//Right wheel forward
const int MOTOR_B2 = 5;//Right wheel backwards

//PULSE SENSOR
const int SENSOR_R1 = 3;  
const int SENSOR_R2 = 2; 
int Signal_R1;
int Signal_R2;          

//ULTRASONIC SENSOR
long distance = 1000;

// defines pins numbers
const int trigPin = 8;
const int echoPin = 11;


//LINE SENSOR VARIABLES
QTRSensors lineSensors;
const int SENSOR_COUNT = 8;
int sensorValues[SENSOR_COUNT];
int pos = 0;

//GRIPPER
const int GRIPPER_PIN=12;
const int GRIPPER_OPEN_PULSE=1600;
const int GRIPPER_CLOSE_PULSE=971;
const int GRIPPER_PULSE_REPEAT=10;


// bool result = false;

//---------------------------------------------------------------------------------------FUNCTION HEADERS-----------------------------------------------------------------------------------
// //MOVING FUNTIONS
void moveForward(int powerA = 255,int powerB = 255);//move the robot forward
void moveBackward(int powerA = 255,int powerB = 255);//move the robot backwards
//---------------------------------------------------------------------------------FUNCTION DECLARATIONS------------------------------------------------------------------------------------
// SETUP AND LOOP FUNTIONS
void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  openGripper();
  leds.begin();
  Serial.begin(9600);
  setup_motor_pins();
  // setupButtons();
  setupLineSensors();
  calibration();
  
  Serial.println("Beginning Race");
  while(!allBlack())
  {
    pos=lineSensors.readLineBlack(sensorValues);
    showSensorValues();
    beginRace();
  }
  Serial.println("Picking up object");
  idle();
  delay(1000);
  closeGripper();
  Serial.println("Picked up the object");
  moveForward(250,250);
  delay(200);
  idle();
  moveForward(0,250);
  delay(700);
  idle();
  Serial.println("Beginning race");
}

void loop() {
  
 // button_control();
  leds.show();
  pos=lineSensors.readLineBlack(sensorValues);
  showSensorValues();
  lineFollow();
  // isObject();

  // if (isObject() == true)
  // {
  //   avoidObject();
  // }


  // if (result == true)
  // {
  
  //   idle();
  //   delay(1000);
  //   turnBeforeObject(250, 250);
  //   delay(250);
  //   moveForward(120, 250);
  //   moveBackward(0,0);
  //   delay(2350);
  //   idle();
  //   delay(500);
  //   turnBeforeObject(250, 250);
  //   delay(105);
  
  // }
  

  // // Clears the trigPin
  // digitalWrite(trigPin, LOW);
  // // delayMicroseconds(2);
  // // Sets the trigPin on HIGH state for 10 micro seconds
  // digitalWrite(trigPin, HIGH);
  // // delayMicroseconds(10);
  // digitalWrite(trigPin, LOW);
  // // Reads the echoPin, returns the sound wave travel time in microseconds
  // long duration = pulseIn(echoPin, HIGH);
  // // Calculating the distance
  // distance = duration * 0.034 / 2;
  // // Prints the distance on the Serial Monitor
  // Serial.println("Distance: " + distance);
}

//MOTOR FUNCTIONS
void idle()
{
  int motor_pins[4]={MOTOR_A1,MOTOR_A2,MOTOR_B1,MOTOR_B2};
  leds.clear();
  for (int i=0;i<4;i++)
    {      
      analogWrite(motor_pins[i],0);
    }
}

void setup_motor_pins()
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


//LINE SENSOR FUNCTIONS
void setupLineSensors()
{
  lineSensors.setTypeAnalog();
  lineSensors.setSensorPins((const uint8_t[]){A0,A1,A2,A3,A4,A5,A6,A7},SENSOR_COUNT); 
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
        idle();
        delay(100);
        moveBackward(152,175);
        for(uint8_t i = 0; i < 25 ; i++)
        {
            lineSensors.calibrate();
            Serial.print("Calibration: ");
            Serial.println(i);
        }
        Serial.println("Calibration done");
        
    }
    //end of the calibration
    idle();
    moveForward(180,205);
    delay(1500);
    //note the maximum and minimum values recorded during the calibration process
    for (int i=0;i<SENSOR_COUNT;i++)
    {
      // Serial.print(lineSensors.calibrationOn.minimum[i]);
      // Serial.print(" ");
    }
    // Serial.println();
    for (int i=0;i<SENSOR_COUNT;i++)
    {
      // Serial.print(lineSensors.calibrationOn.maximum[i]);
      // Serial.print(" ");
    }
    idle();
}
void showSensorValues()
{
  for (int i=0;i<SENSOR_COUNT;i++)
    {
      // Serial.print(isBlack(sensorValues[i]));
      // Serial.print(" ");
    }
  //  Serial.println();
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

bool mostlyBlack()
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
  if (allBlack() )
  {
    // idle();
    moveForward(200,200);
    delay(200);
    // idle();
  }
  else if (allWhite())
    {
      moveForward(200, 200);
      // Serial.println("Finished Track");
    }
  else if (lineAhead())
  {
    moveForward(200,200);
  }
  else  if (lineRight())
  {
    // idle();
    moveForward(200,0);
  }
  else if (lineLeft())
  {
    // idle();
    moveForward(0,200);
  }
  else if (mostlyBlack())
  {
    // idle();
    moveForward(200,200);
    delay(200);
    // idle();
  }
  else
  {
    // idle();
  }
}

void beginRace()
{
  if (lineAhead())
  {
    moveForward(200,200);
  }
  else  if (lineRight())
  {
    idle();
    moveForward(200,0);
  }
 else if (lineLeft())
  {
    idle();
    moveForward(0,200);
  }
 else if (allWhite())
 {
  moveForward(200,200);
 }
}

// bool isObject()
// {
//   if (distance <= 16)
//   {
//      result = true;
//   }
//   else
//   {
//     result = false;
//   }
// }

// void avoidObject()
// {
//   // if (isObject() == true)
//   // {
//     idle();
//     delay(1000);
//     turnBeforeObject(250, 250);
//     delay(250);
//     moveForward(120, 250);
//     moveBackward(0,0);
//     delay(2350);
//     idle();
//     delay(500);
//     turnBeforeObject(250, 250);
//     delay(105);
//   // }
// }

void turnBeforeObject(int powerA, int powerB)
{
  analogWrite(MOTOR_A2, powerA);
  analogWrite(MOTOR_B2, powerB);
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
