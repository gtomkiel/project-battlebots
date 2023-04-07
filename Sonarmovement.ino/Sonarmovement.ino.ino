/** THIS PROGRAM IS FOR RACE CAR EVADING COLLISION
   HC-SR04 has a range 2cm to 400cm
The sensor is composed of two ultrasonic transducers. One is transmitter which outputs ultrasonic sound pulses and the other is receiver which listens for reflected waves. 
It’s basically a SONAR which is used in submarines for detecting underwater objects.

Trig wire-> Green wire sends  ultrasonic wave
Echo wire-> Yellow wire listens for reflected signal
Considering the travel time and the speed of the sound you can calculate the distance.
===================[CURRENT TASKS]===================
  add a method that ensures car stays in the middle of the track by keeping a distance to the walls while moving

===================[FUTURE TASKS]===================

*/

//===== [Initialising Adafruit library] =====
#include <Adafruit_NeoPixel.h>
#include <QTRSensors.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

//===== [Gripper Rotation] ===== 
#define gripperOpenPulse 1600
#define gripperClosePulse 971
#define gripperPin 9
#define gripperPulseRepeat 10

// ===== [PINS] =====
#define frontSonarSensorTrigPin 7
#define frontSonarSensorEchoPin 4

#define leftSonarSensorTrigPin 13
#define leftSonarSensorEchoPin 12



#define leftTireForward 5
#define leftTireBackward 11 
#define rightTireForward 6
#define rightTireBackward 10

#define leftWheelEncoder 2 //R1 encoder
#define rightWheelEncoder 3  //R2 encoder

QTRSensors qtr;
const uint8_t SensorCount = 8;

uint16_t sensorValues[SensorCount];

//===================[QTR 8 SENSOR]============================
/*
  w1- a4
  w2- a5
  purp-a6
  blue-a7
  green-a0
  yellow- a1
  orange- a2
  brown-a3
*/
#define irSensorOne A4
#define irSensorTwo A5
#define irSensorThree A6
#define irSensorFour A7
#define irSensorFive A0
#define irSensorSix A1
#define irSensorSeven A2
#define irSensorEight A3

//================[NEOPIXEL]========================
#define pixelPIN 8      // pin assigned to NI of neoPixels
#define NUMPIXELS 4   //number of pixels attached to strip

/* Parameter 1 = number of pixels in strip
   Parameter 2 = Arduino pin number (most are valid)
   Parameter 3 = pixel type flags, add together as needed:
   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
*/
Adafruit_NeoPixel strip(NUMPIXELS, pixelPIN, NEO_GRB + NEO_KHZ800);

//define variables
long duration;
int distance;

volatile int countRW = 0;
volatile int countLW = 0;

const double cDistance = 13; 
const double leftUpperLimitDistance = 30;
const double leftLowerLimitDistance = 15;//15
int fullLineCounter = 0;
double fDistance = 0;
double leftSonarDistance = 0;
boolean startBot = false;
//Define time for events
unsigned long previousMillis_1 = 0; //Stores time for 1st event
unsigned long previousMillis_2 = 0; //stores time for 2nd event

//If sense infront turn right, if after millis still sense, turn left,while still sense turn left,after go forward
const long interval_1 = 20000; //interval to be used for turning
const long interval_2 = 2500; //interval to turn backwards
const long interval_3 = 3000; //interval to swap from turnLeft to moveForward
//after backwards, turn left, after an interval move forward


//===== [SETUP] =====
void setup(){
 // This is for Trinket 5V 16MHz, the three lines are removable if you are not using a Trinket
  #if defined (_AVR_ATtiny85_)
  if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
  #endif
 // End of trinket special code

  strip.begin();            //start strip
  strip.setBrightness(50);  //set brightness of strip
  strip.show();             //Initialise all pixels to 'off' 

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A4,A5,A6,A7,A0,A1,A2,A3}, SensorCount);

  pinMode(frontSonarSensorTrigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(frontSonarSensorEchoPin, INPUT); // Sets the echoPin as an Input
  pinMode(leftSonarSensorTrigPin, OUTPUT);
  pinMode(leftSonarSensorEchoPin, INPUT);
  
  Serial.begin(9600); // Starts the serial communication

  //Setting up motor pins
  pinMode (leftTireForward, OUTPUT);
  pinMode (leftTireBackward, OUTPUT);
  pinMode (rightTireForward, OUTPUT);
  pinMode (rightTireBackward, OUTPUT);

  digitalWrite(leftTireForward, LOW);
  digitalWrite(leftTireBackward, LOW);
  digitalWrite(rightTireForward, LOW);
  digitalWrite(rightTireBackward, LOW);
  
  //Setting up servoPins
  pinMode(gripperPin, OUTPUT);

  pinMode(irSensorOne, INPUT);
  pinMode(irSensorTwo, INPUT);
  pinMode(irSensorThree, INPUT);
  pinMode(irSensorFour, INPUT);
  pinMode(irSensorFive, INPUT);
  pinMode(irSensorSix, INPUT);
  pinMode(irSensorSeven, INPUT);
  pinMode(irSensorEight, INPUT);

  //Setup encoders
  attachInterrupt(digitalPinToInterrupt(rightWheelEncoder), updateRightWheel, CHANGE);
  attachInterrupt(digitalPinToInterrupt(leftWheelEncoder), updateLeftWheel, CHANGE);
}

//int number = 0;

//===== [LOOP] =====
void loop(){
qtr.read(sensorValues);
  unsigned long currentMillis_1 = millis();
 
  Serial.print("Time is: ");
  Serial.println(currentMillis_1);
  openGripper();
  wait(1500);
  if(puckPlaced()){
    basicMoveForward();
    delay(1300); 
    while(!allBlack()){
      lineSensorMoveForward();
    }
    basicMoveForward();
    wait(240);
    clearMotors();
    closeGripper();
    wait(100);
    clearLineSensorValues();
    rotateLeft();
    basicMoveForward();
    delay(2000);   //1600
    startBot = true;
    while(startBot){
      mazeMoveForwardWithTicks();
      clearMotors();
      if(forwardDistance() < cDistance){
        if(leftSonarSensorDistance() < leftUpperLimitDistance){
          rotateRight();
          clearMotors();
          wait(300);
          if(forwardDistance() < cDistance){ 
//        wait(300);
          rotateRight();
          }
        }  
      }
      if(leftSonarSensorDistance() > leftUpperLimitDistance){
        clearMotors();
        wait(300);
        moveForwardInTicks(39);//39 //44
        rotateLeft();
        clearMotors();
        wait(300);
        moveForwardInTicks(46);//46
      }
     if(currentMillis_1 - previousMillis_1 >= interval_1 ){
        previousMillis_1 = millis();
        if(lineSensorActivated()){

        while(!allBlack()){
          lineSensorMoveForward();
        }
        clearMotors();
        openGripper();
        wait(300);
        moveBackward();
        delay(600);
        startBot = false;
        clearMotors();
        wait(4000000);
      }
     }
   }
  }   
}

void clearLineSensorValues(){
  
  for(int i = 0; i < SensorCount ; i++){

    sensorValues[i] = 0;
  } 
  
}

boolean lineSensorActivated(){
  
  if(sensorValues[0]>750 || sensorValues[1]>750 || sensorValues[2] >750 || sensorValues[3]>750 || sensorValues[4]>750 || sensorValues[5]>750 || sensorValues[6]>750 || sensorValues[7]>750){
      
    return true;
  }
  
  return false;
}

//hardcode 25cm at the start of bot, 
void lineSensorMoveForward(){
  qtr.read(sensorValues);
 
//sensorValues[2] >700 && sensorValues[3]>700 && sensorValues[4]>700 && sensorValues[5]>700
//  printLineSensorValues();

  if(sensorValues[3]>600 && sensorValues[4]>600){

    basicMoveForward();
    
  }else if(sensorValues[2]>600 && sensorValues[3]>600){
    
    lineAdjustRight();
  }else if(sensorValues[0]>600 || sensorValues[1]>600){

    lineAdjustRight();
  }else if(sensorValues[5] >600 && sensorValues[4]>600){
    
    lineAdjustLeft();
  }else if(sensorValues[6]>600 || sensorValues[7] > 600){
    
    lineAdjustLeft();
   }else{
    
    clearMotors();  
  }
}

boolean allBlack(){

  if(sensorValues[0]>600 && sensorValues[1]>600 && sensorValues[2] >600 && sensorValues[3]>600 && sensorValues[4]>600 && sensorValues[5]>600 && sensorValues[6]>600 && sensorValues[7]>600){
    return true;
  }
  return false;  
}

void printLineSensorValues(){
  
  for(int i = 0; i < SensorCount ; i++){

    Serial.print(sensorValues[i]);
    Serial.print(" ");
    if(i == 7){
      Serial.println(" "); 
    }
  }  
}

void start(){
  qtr.read(sensorValues);
  unsigned long currentMillis_1 = millis();
 
  Serial.print("Time is: ");
  Serial.println(currentMillis_1);
  openGripper();
  wait(1500);
  if(puckPlaced()){
    basicMoveForward();
    delay(1300); 
    while(!allBlack()){
      lineSensorMoveForward();
    }
    basicMoveForward();
    wait(240);
    clearMotors();
    closeGripper();
    wait(100);
    clearLineSensorValues();
    rotateLeft();
    basicMoveForward();
    delay(1600);
    startBot = true;
    while(startBot){
      mazeMoveForwardWithTicks();
      clearMotors();
      if(forwardDistance() < cDistance){
        if(leftSonarSensorDistance() < leftUpperLimitDistance){
          rotateRight();
          clearMotors();
          wait(300);
          if(forwardDistance() < cDistance){ 
//        wait(300);
          rotateRight();
          }
        }  
      }
      if(leftSonarSensorDistance() > leftUpperLimitDistance){
        clearMotors();
        wait(300);
        moveForwardInTicks(39);//39 //44
        rotateLeft();
        clearMotors();
        wait(300);
        moveForwardInTicks(46);//46
      }
     if(currentMillis_1 - previousMillis_1 >= interval_1 ){
        previousMillis_1 = millis();
        if(lineSensorActivated()){

        while(!allBlack()){
          lineSensorMoveForward();
        }
        clearMotors();
        openGripper();
        wait(300);
        moveBackward();
        delay(600);
        startBot = false;
        clearMotors();
        wait(4000000);
      }
     }
   }
  }
}

void lineAdjustRight(){

  analogWrite(leftTireForward,200);   // turns left tire forward
  analogWrite(rightTireForward,130);  //turns right tire forward
  analogWrite(leftTireBackward, 0);   //left tire backward 0
  analogWrite(rightTireBackward, 0);   //right tire backward 0
}

void lineAdjustLeft(){

  analogWrite(leftTireForward,130);   // turns left tire forward
  analogWrite(rightTireForward,200);  //turns right tire forward
  analogWrite(leftTireBackward, 0);   //left tire backward 0
  analogWrite(rightTireBackward, 0);   //right tire backward 0
}

boolean puckPlaced(){

  if(forwardDistance() < 35){           //If detect car/puck
      return true;
  }
  return false;
}


void mazeMoveForwardWithTicks(){

        while(leftSonarSensorDistance() <= leftLowerLimitDistance && leftSonarSensorDistance() > 1 && countLW < 100000000000 && forwardDistance() > cDistance){ //Fix adjeust when out of lowerLimit
        //rangeOfMoveForward 
        if(leftSonarSensorDistance() > 8){//8 //7.2
         
          rotateLessLeft(2);//2 //1
          Serial.println("Car adjustedLeft");
//          basicMoveForward();
          neoTurnLeft();
        }else if(leftSonarSensorDistance() > 6.4 && leftSonarSensorDistance() <= 7.2){//6-8 //6.4-7.2
          
          analogWrite(leftTireForward,180);   // turns left tire forward
          analogWrite(rightTireForward,220);  //turns right tire forward 200
          analogWrite(leftTireBackward, 0);   //left tire backward 0
          analogWrite(rightTireBackward, 0);   //right tire backward 0
          Serial.println("Car adjustedLeft");
          neoRotateLessLeft();
        }
        else if(leftSonarSensorDistance() < 4.7 && leftSonarSensorDistance() >= 4.0){ //4-4.7 //2-2.8
          
          analogWrite(leftTireForward,220);   // turns left tire forward 200
          analogWrite(rightTireForward,180);  //turns right tire forward
          analogWrite(leftTireBackward, 0);   //left tire backward 0
          analogWrite(rightTireBackward, 0);   //right tire backward 0
          Serial.println("Car adjustedRight");
          neoRotateLessRight();
        }else if(leftSonarSensorDistance() < 4.0 && leftSonarSensorDistance() >= 1.3){//4 //2.5
          
          rotateLessRight(2);//2 //1
          Serial.println("Car adjustedRight");
          neoTurnRight();        
        }else{//leftSonarSensorDistance between 4.0-60 moveForward
            analogWrite(leftTireForward,255);   // turns left tire forward
            analogWrite(rightTireForward,255);  //turns right tire forward
            analogWrite(leftTireBackward, 0);   //left tire backward 0
            analogWrite(rightTireBackward, 0);   //right tire backward 0
            Serial.println("Car movedForward");
            neoMoveForward();
          }
       }  
       if(leftSonarSensorDistance() < 1.3 && forwardDistance() > cDistance){
        
        moveBackward();
        delay(120);
        Serial.println("stuck, car moveBackwards");
       }
       
       if(leftSonarSensorDistance() > leftLowerLimitDistance){
        
          rotateLessLeft(12); //12 //8
          neoRotateLessLeftt();
          Serial.println("Car steeredLeft hard 12");
       }
}

void rotateLeft90(){
  
  int ticks = 50;
  resetCounters();
  while(countLW < ticks ){
    
    analogWrite(leftTireBackward,200);  //left tire backward
    analogWrite(rightTireForward,200);  //right tire forward
    analogWrite(leftTireForward, 0);   //left tire forward 0
    analogWrite(rightTireBackward, 0);   //right tire backward 0
    neoTurnLeft();
  }
}

void rotateRight90(){

 int ticks = 48;
 resetCounters();
  while(countRW < ticks){
    analogWrite(leftTireForward,200);  //left tire forward
    analogWrite(rightTireBackward,200);  //right tire backward
    analogWrite(leftTireBackward, 0);   //left tire backward 0
    analogWrite(rightTireForward, 0);   //right tire forward 0
    neoTurnRight();
  }  
}

void rotateLessLeft(int ticks){

 resetCounters();
  while(countLW < ticks){
    analogWrite(leftTireBackward,200);  //left tire backward
    analogWrite(rightTireForward,200);  //right tire forward
    analogWrite(leftTireForward, 0);   //left tire forward 0
    analogWrite(rightTireBackward, 0);   //right tire backward 0
    neoTurnLeft();
  }  
  //clearMotors();
}

void rotateLessRight(int ticks){

 resetCounters();
  while(countRW < ticks){
    analogWrite(leftTireForward,200);  //left tire forward
    analogWrite(rightTireBackward,200);  //right tire backward
    analogWrite(leftTireBackward, 0);   //left tire backward 0
    analogWrite(rightTireForward, 0);   //right tire forward 0
    neoTurnRight();
  }  
  //clearMotors();
}

// counter functions
// used to update the counters of the encoders

void updateRightWheel()
{
    noInterrupts();
    countRW++;
    interrupts();
}

void updateLeftWheel()
{
    noInterrupts();
    countLW++;
    interrupts();
}

void resetCounters()
{
    countRW = 0;
    countLW = 0;
}

void gripperServo(int pulse){

  for(int i = 0; i < gripperPulseRepeat; i++){

    digitalWrite(gripperPin,HIGH);
    delayMicroseconds(pulse); //in microseconds
    digitalWrite(gripperPin,LOW);
    delay(20);    
  }
}

void openGripper(){

  gripperServo(gripperOpenPulse);
}

void closeGripper(){

  gripperServo(gripperClosePulse);
}

void moveForwardInTicks(int ticks){

  resetCounters();

  while(countLW < ticks){

    analogWrite(leftTireForward,200);   // turns left tire forward
    analogWrite(rightTireForward,200);  //turns right tire forward
    analogWrite(leftTireBackward, 0);   //left tire backward 0
    analogWrite(rightTireBackward, 0);   //right tire backward 0
    }
  //clearMotors();
}

void basicMoveForward(){
  //Serial.print("Left sensor distance: ");
    //Serial.println(calculateLeftSonarSensorDistance());    
    analogWrite(leftTireForward,180);   // turns left tire forward
    analogWrite(rightTireForward,180);  //turns right tire forward
    analogWrite(leftTireBackward, 0);   //left tire backward 0
    analogWrite(rightTireBackward, 0);   //right tire backward 0
    neoMoveForward();
}

void rotateRight(){

  int counter = 60;

  for(int i = 0; i < counter; i++){

    turnRight();
    delay(9);
  }
}

void rotateLeft(){

  int counter = 60;

  for(int i = 0; i < counter; i++){

    turnLeft();
    delay(9);
  }
}
void turnRight(){ //This moves the my left tire forwards and stops the right tire from rotating
  analogWrite(leftTireForward,200);  //left tire forward
  analogWrite(rightTireBackward,200);  //right tire backward
  analogWrite(leftTireBackward, 0);   //left tire backward 0
  analogWrite(rightTireForward, 0);   //right tire forward 0
  neoTurnRight();
}
  
void turnLeft(){ //This moves the my left tire backwards and stops the right tire from rotating
  analogWrite(leftTireBackward,200);  //left tire backward
  analogWrite(rightTireForward,200);  //right tire forward
  analogWrite(leftTireForward, 0);   //left tire forward 0
  analogWrite(rightTireBackward, 0);   //right tire backward 0
  neoTurnLeft();
}

void moveBackward(){ //Moves both tires backwards
  analogWrite(rightTireBackward,200);  //right tire backward
  analogWrite(leftTireBackward,200);  //turns left tire backward
  analogWrite(rightTireForward,0);    //right tire forward 0
  analogWrite(leftTireForward,0);    //left forward 0
  neoMoveBackward();
}

void clearMotors(){ //Stops motor by setting both tires to the same signal
  analogWrite(rightTireBackward,0);   // truerns right tire 0
  analogWrite(leftTireBackward,0); //turns left tire 0
  analogWrite(rightTireForward,0);   // truerns right tire 0
  analogWrite(leftTireForward,0); //turns left tire 0
}

void neoMoveForward(){
  int n1 = 0;       //backleft
  int n2 = 1;       //backright
  int n3 = 2;        //topright
  int n4 = 3;       //topleft
   strip.setPixelColor(n4,255, 0,0); //means topleft is set to green (g,r,b)
   strip.setPixelColor(n3, 255, 0 , 0); //means topright is set to green (g,r,b)
   strip.setPixelColor(n2, 0, 0 , 0); //means backright is set to green (g,r,b)
   strip.setPixelColor(n1, 0, 0 , 0); //means backleft is set to green (g,r,b)
   strip.show();
}

void neoTurnRight(){
  int n1 = 0;       //backleft
  int n2 = 1;       //backright
  int n3 = 2;        //topright
  int n4 = 3;       //topleft
   strip.setPixelColor(n4,0, 0,0); //means topleft is set to green (g,r,b)
   strip.setPixelColor(n3, 255, 0 , 0); //means topright is set to green (g,r,b)
   strip.setPixelColor(n2, 0, 0 , 0); //means backright is set to green (g,r,b)
   strip.setPixelColor(n1, 0, 0 , 0); //means backleft is set to green (g,r,b)
   strip.show();
}

void neoRotateLessRight(){
  int n1 = 0;       //backleft
  int n2 = 1;       //backright
  int n3 = 2;        //topright
  int n4 = 3;       //topleft
   strip.setPixelColor(n4, 0, 0, 0); //means topleft is set to green (g,r,b)
   strip.setPixelColor(n3, 0, 0 , 255); //means topright is set to green (g,r,b)
   strip.setPixelColor(n2, 0, 0 , 0); //means backright is set to green (g,r,b)
   strip.setPixelColor(n1, 0, 0 , 0); //means backleft is set to green (g,r,b)
   strip.show();
}

void neoRotateLessRightt(){
  int n1 = 0;       //backleft
  int n2 = 1;       //backright
  int n3 = 2;        //topright
  int n4 = 3;       //topleft
   strip.setPixelColor(n4, 0, 0, 0); //means topleft is set to green (g,r,b)
   strip.setPixelColor(n3, 0, 255 , 0); //means topright is set to green (g,r,b)
   strip.setPixelColor(n2, 0, 0 , 0); //means backright is set to green (g,r,b)
   strip.setPixelColor(n1, 0, 0 , 0); //means backleft is set to green (g,r,b)
   strip.show();
}

void neoRotateLessLeftt(){
  int n1 = 0;       //backleft
  int n2 = 1;       //backright
  int n3 = 2;        //topright
  int n4 = 3;       //topleft
   strip.setPixelColor(n4, 0, 0, 0); //means topleft is set to green (g,r,b)
   strip.setPixelColor(n3, 0,255, 0); //means topright is set to green (g,r,b)
   strip.setPixelColor(n2, 0, 0 , 0); //means backright is set to green (g,r,b)
   strip.setPixelColor(n1, 0, 0 , 0); //means backleft is set to green (g,r,b)
   strip.show();
}

void neoTurnLeft(){
  int n1 = 0;       //backleft
  int n2 = 1;       //backright
  int n3 = 2;        //topright
  int n4 = 3;       //topleft
   strip.setPixelColor(n4,255, 0,0); //means topleft is set to green (g,r,b)
   strip.setPixelColor(n3, 0, 0 , 0); //means topright is set to green (g,r,b)
   strip.setPixelColor(n2, 0, 0 , 0); //means backright is set to green (g,r,b)
   strip.setPixelColor(n1, 0, 0 , 0); //means backleft is set to green (g,r,b)
   strip.show();
}
void neoRotateLessLeft(){
  int n1 = 0;       //backleft
  int n2 = 1;       //backright
  int n3 = 2;        //topright
  int n4 = 3;       //topleft
   strip.setPixelColor(n4,0, 0,255); //means topleft is set to green (g,r,b)
   strip.setPixelColor(n3, 0, 0 , 0); //means topright is set to green (g,r,b)
   strip.setPixelColor(n2, 0, 0 , 0); //means backright is set to green (g,r,b)
   strip.setPixelColor(n1, 0, 0 , 0); //means backleft is set to green (g,r,b)
   strip.show();
}


void neoMoveBackward(){
  int n1 = 0;       //backleft
  int n2 = 1;       //backright
  int n3 = 2;        //topright
  int n4 = 3;       //topleft 
   strip.setPixelColor(n4,0, 0,0); //means topleft is set to green (g,r,b)
   strip.setPixelColor(n3, 0, 0 , 0); //means topright is set to green (g,r,b)
   strip.setPixelColor(n2, 0, 255 , 0); //means backright is set to green (g,r,b)
   strip.setPixelColor(n1, 0, 255 , 0); //means backleft is set to green (g,r,b)
   strip.show();
}

double forwardDistance(){
   // Clears the trigPin
  digitalWrite(frontSonarSensorTrigPin, LOW);
  delayMicroseconds(2);

  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(frontSonarSensorTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(frontSonarSensorTrigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(frontSonarSensorEchoPin, HIGH);

  // Calculating the distance
  int distance = duration * 0.034 / 2;

  fDistance = distance;

  return distance;
}

void printForwardDistance(){ //Prints the distance calculated
// Prints the distance on the Serial Monitor
  Serial.print("ForwardDistance: ");
  Serial.println(forwardDistance());
}

double leftSonarSensorDistance(){
  
  // Clears the trigPin
  digitalWrite(leftSonarSensorTrigPin, LOW);
  delayMicroseconds(2);

  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(leftSonarSensorTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(leftSonarSensorTrigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(leftSonarSensorEchoPin, HIGH);

  // Calculating the distance
  double LeftSonarSensorDis = duration * 0.034 / 2;

  // leftSonarSensorDistance = calculatedleftSonarSensor;

  return LeftSonarSensorDis; 
}

void updateLeftSonarSensorDistance(){

  leftSonarDistance = leftSonarSensorDistance();
}

void wait(int timeToWait)
// waits for an amount of time in milliseconds
// used to eliminate the need to use the delay() function
{
    long time = millis();
    while (millis() < time + timeToWait);
}
