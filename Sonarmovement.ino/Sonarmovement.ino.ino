/** THIS PROGRAM IS FOR RACE CAR EVADING COLLISION
   HC-SR04 has a range 2cm to 400cm
The sensor is composed of two ultrasonic transducers. One is transmitter which outputs ultrasonic sound pulses and the other is receiver which listens for reflected waves. 
It’s basically a SONAR which is used in submarines for detecting underwater objects.

Trig wire-> Green wire sends  ultrasonic wave
Echo wire-> Yellow wire listens for reflected signal
Considering the travel time and the speed of the sound you can calculate the distance.
===================[CURRENT TASKS]===================
  add a method that ensures car stays in the middle of the track by keeping a distance to the walls while moving
  add a method that makes car turnLeft if leftSonarSensor_Distance > upperLimit_Distance, meaning if it senses a lot of space to the left
  if leftSonarSensor_Distance < competentSideDistance, moveForward. If leftSonarSensor_Distance > competentSideDistance &&  leftSonarSensor_Distance < upperLimit_Distance
  turnLeft till leftSonarSensor_Distance < competentSideDistance
  could use a while loop, while()=> continuosForward, evadeCollision5
  if sideDistance > upperLimit, turnLeft/turnRight
  create pulses method for moveForward & turndirections

===================[FUTURE TASKS]===================
  Then bluetooth configuration(Search bluetooth 2 way connector)
  Learn how to use bluetooth to activate robot(relay race related)

  w1- a4
  w2- a5
  purp-a6
  blue-a7
  green-a0
  yellow- a1
  orange- a2
  brown-a3
*/

//===== [Initialising Adafruit library] =====
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

//===== [Gripper Rotation] ===== 
#define gripperOpenPulse 1600
#define gripperClosePulse 971

// ===== [PINS] =====
#define frontSonarSensorTrigPin 7
#define frontSonarSensorEchoPin 4

#define leftSonarSensorTrigPin 13
#define leftSonarSensorEchoPin 12

#define gripperPin 9

#define leftTireForward 5
#define leftTireBackward 11 
#define rightTireForward 6
#define rightTireBackward 10

#define leftWheelEncoder 2 //R1 encoder
#define rightWheelEncoder 3  //R2 encoder

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

//A1 - D11
//A2 - D5
//B1 - D6
//B2 - D10
//r1-rIGHTwHEEL-d2
//r2-leftwheel-d3

volatile int countRW = 0;
volatile int countLW = 0;

const int cDistance = 13; //13
int fDistance = 0;

double leftSonarSensorDistance = 0;
//Define time for events
unsigned long previousMillis_1 = 0; //Stores time for 1st event
unsigned long previousMillis_2 = 0; //stores time for 2nd event

//If sense infront turn right, if after millis still sense, turn left,while still sense turn left,after go forward
const long interval_1 = 100; //interval to be used for turning
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

  //Setup encoders

  attachInterrupt(digitalPinToInterrupt(rightWheelEncoder), updateRightWheel, CHANGE);
  attachInterrupt(digitalPinToInterrupt(leftWheelEncoder), updateLeftWheel, CHANGE);
}


/*
if(puckPlaced()){
  openGripper();
  delay(15000);   //delay 15 seconds
  start();
}


boolean puckPlaced(){

  if(forwardDistance() < 30){
    return true;
  }
  return false;
}

boolean mazeStart->true
start(){
  while(mazeStart == true){

    pulseForward(20);
    closeGripper();
    
    rotateLeft();
    continuousForward();
    .....................
  }
}
*/


int number = 0;

//===== [LOOP] =====
void loop(){
  
  Serial.print("LOOP: ");
  Serial.println(number);
//    discoverTires();
   moveForwardInTicks(50);
  Serial.println(countLW);
  clearMotors();
  wait(1000);
//  moveForwardInTicks(2);
//   wait(4000);
//   moveForwardInTicks(5);
//   wait(4000);
//   moveForwardInTicks(10);
//   wait(4000);
   //printDistance();
 // testRotateRightCollision();
 
 number+=1;
 
}

  void discoverTires(){

    analogWrite(leftTireForward,0);     // movesLeftTireBackward
    analogWrite(rightTireForward,0);  //movesRightTireForward
    analogWrite(leftTireBackward, 0);   //movesLeftTireForward
    analogWrite(rightTireBackward, 160);  //
    
  }

void rotateRight180(){
  
  int counter = 80;
  for(int i = 0; i < counter; i++){

    turnRight();
    delay(9);
  }
 // clearMotors();
  //delay(300000000);
}

void rotateLeft90(){
  int counter = 90;
  for(int i = 0; i < counter; i++){

    turnLeft();
    delay(5);
  }
  clearMotors();
  //delay(300000000);
}


void rotateRight90(){

 int counter = 90;
  for(int i = 0; i < counter; i++){

    turnRight();
    delay(5);
  }
  clearMotors();
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


/*
  create pulseForward() method

*/

/*
 while(calcLeftDistance() < 30){
  
  continuousForward();

  if(forwardDistance() < cDistance && calcLeftDistance() < 30){

    rotateRight90;
    pulseForward(20); //20 pulses
  } 
 }
  if(calcLeftDistance() >= 30){
    
  pulseForward(20);   //20 pulses
  rotateLeft90();
  pulseForward(20);   //20 pulses
  }  
 }
*/
//
//void evadeCollision6(){
//  
//  lookLeft();
//  Serial.println("I looked left");
//  if(leftDistance() > cDistance){
//
//     turnLeft(); 
//     Serial.println("I turned left");
//    }else if(leftDistance() <= cDistance){
//      
//      lookRight();
//      //delay(40);
//      Serial.println("I looked right");
//      if(rightDistance() > cDistance){
////      rotateRight90();
//      turnRight(); 
//
//      Serial.println("I turned right");
//      } else{ //Use millis
//
//         //rotateRight180();
//         Serial.println("I rotated 180right");
//      }  
//    }
//     lookForward();
//}





void continuousForward(){

  //printDistance();
 
//  delay(200);
  while(calculateDistance() > cDistance){
     mazeMoveForward();
    // neoMoveForward();
   //  Serial.println("calculateDistance() higher");
    }
   
    //Serial.println("cDistance higher");
}


void servo(int pin, int length){
  
  digitalWrite(pin,HIGH);
  delayMicroseconds(length); //in microseconds
  digitalWrite(pin,LOW);
  delay(20);
}

void gripperOpen(){
  
  servo(gripperPin, gripperOpenPulse);
}

void gripperClose(){

  servo(gripperPin, gripperClosePulse);
}

/*
  Current code logic
  moveForward()
  if the distance < cDistance
  turnRight()
  if after interval_1 ditance < cDistance
  turnLeft() while distance < cDistance
  if turnLeft() works, we moveForward()
  
  Now to add a backwards case to our logic
  if stuck in turnLeft for an amount of time,goBackwards, after an amount of time,break
  then moveForward;
  
*/


void evadeCollision(){//Movement logic using millis
 
 unsigned long currentMillis_1 =millis();
 printDistance();
 
  Serial.print("Time is:");
  Serial.println(currentMillis_1);
   clearMotors();
   mazeMoveForward();
   if(calculateDistance()<= cDistance){ //if distance<30 
   // printDistance();
    clearMotors();
    turnRight();                //device turns right 
    
    if(currentMillis_1 - previousMillis_1 >= interval_1 ){  //if after 0,5s
         
          previousMillis_1 = millis();
          if(calculateDistance()<= cDistance){                       // and distance is still less than 60, turn left
            clearMotors();
              while(calculateDistance() < cDistance){        //while loop to ensure the device turns left completely
                 Serial.println("Stuck in while loop");
                turnLeft();
                printDistance();
              }
            }
           clearMotors();
          mazeMoveForward();
      }
    }
  }

void moveForwardInTicks(int ticks){

  resetCounters();

  while(countLW < ticks){

    analogWrite(leftTireForward,180);   // turns left tire forward
    analogWrite(rightTireForward,180);  //turns right tire forward
    analogWrite(leftTireBackward, 0);   //left tire backward 0
    analogWrite(rightTireBackward, 0);   //right tire backward 0
    }
  //clearMotors();
}

void mazeMoveForward(){
  //Serial.print("Left sensor distance: ");
    //Serial.println(calculateLeftSonarSensorDistance());
  if (calculateLeftSonarSensorDistance() <= 5 && calculateLeftSonarSensorDistance() > 1){
    
    analogWrite(leftTireForward,160);   // turns left tire forward
    analogWrite(rightTireForward,160);  //turns right tire forward
    analogWrite(leftTireBackward, 0);   //left tire backward 0
    analogWrite(rightTireBackward, 0);   //right tire backward 0
    neoMoveForward();
    } else (
      clearMotors()); 
  }

  void sMoveForward(){
  //Serial.print("Left sensor distance: ");
    //Serial.println(calculateLeftSonarSensorDistance());
//  if (calculateLeftSonarSensorDistance() <= 7 && calculateLeftSonarSensorDistance() > 1){
    
    analogWrite(leftTireForward,130);   // turns left tire forward
    analogWrite(rightTireForward,130);  //turns right tire forward
    analogWrite(leftTireBackward, 0);   //left tire backward 0
    analogWrite(rightTireBackward, 0);   //right tire backward 0
    neoMoveForward();
//    } else (
//      clearMotors()); 
  }



void turnRight(){ //This moves the my left tire forwards and stops the right tire from rotating
  analogWrite(leftTireForward,160);  //left tire forward
  analogWrite(rightTireBackward,160);  //right tire backward
  analogWrite(leftTireBackward, 0);   //left tire backward 0
  analogWrite(rightTireForward, 0);   //right tire forward 0
  neoTurnRight();
}
  
void turnLeft(){ //This moves the my left tire backwards and stops the right tire from rotating
  analogWrite(leftTireBackward,160);  //left tire backward
  analogWrite(rightTireForward,160);  //right tire forward
  analogWrite(leftTireForward, 0);   //left tire forward 0
  analogWrite(rightTireBackward, 0);   //right tire backward 0
  neoTurnLeft();
}

void moveBackward(){ //Moves both tires backwards
  analogWrite(rightTireBackward,160);  //right tire backward
  analogWrite(leftTireBackward,160);  //turns left tire backward
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

int calculateDistance(){ //Calculates distance to the sonar sensor
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

  return distance;
}



void forwardDistance(){
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
}



double calculateLeftSonarSensorDistance(){
  
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
   double calculatedleftSonarSensor = duration * 0.034 / 2;

  // leftSonarSensorDistance = calculatedleftSonarSensor;

  return calculatedleftSonarSensor;
  
}
void printDistance(){ //Prints the distance calculated
// Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.println(calculateDistance());
}

void wait(int timeToWait)
// waits for an amount of time in milliseconds
// used to eliminate the need to use the delay() function
{
    long time = millis();

    while (millis() < time + timeToWait)
        ;
}

// sMoveForward();
//  if(currentMillis_1 - previousMillis_1 >= interval_1 ){  //if after 0,6s, run this
//    previousMillis_1 = currentMillis_1;
//   continuousForward();
//   evadeCollision6();
//  }
