/** THIS PROGRAM IS FOR RACE CAR EVADING COLLISION
   HC-SR04 has a range 2cm to 400cm
The sensor is composed of two ultrasonic transducers. One is transmitter which outputs ultrasonic sound pulses and the other is receiver which listens for reflected waves. 
It’s basically a SONAR which is used in submarines for detecting underwater objects.

Trig wire-> Green wire sends  ultrasonic wave
Echo wire-> Yellow wire listens for reflected signal
Considering the travel time and the speed of the sound you can calculate the distance.
  r1 - 12 
  r2 - 13
===================[CURRENT TASKS]===================
  add a method that ensures car stays in the middle of the track by keeping a distance to the walls while moving
  add a method that makes car turnLeft if leftSonarSensor_Distance > upperLimit_Distance, meaning if it senses a lot of space to the left
  if leftSonarSensor_Distance < competentSideDistance, moveForward. If leftSonarSensor_Distance > competentSideDistance &&  leftSonarSensor_Distance < upperLimit_Distance
  turnLeft till leftSonarSensor_Distance < competentSideDistance
  could use a while loop, while()=> continuosForward, evadeCollision5
  if sideDistance > upperLimit, turnLeft/turnRight

===================[FUTURE TASKS]===================
  Then bluetooth configuration(Search bluetooth 2 way connector)
  Learn how to use bluetooth to activate robot(relay race related)
*/


//Initialising use of Ardafruit
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

//Initialising Servo
#define sonarServoPin 10 
#define gripperPin 9

//defining sonarServo rotation in microseconds
#define sonarServoMinPulse 500
#define sonarServoCenterPulse 1520
#define sonarServoMaxPulse 2400
#define servoPulseRepeat 10   //number of pulses sent to servo,A servo needs at least a minimum of 2 pulses but generally 10 pulses.

//defining gripper rotation 
#define gripperOpenPulse 1600
#define gripperClosePulse 971

// define sonar sensor pins
#define frontSonarSensorTrigPin 7       //trigPin
#define frontSonarSensorEchoPin 6       //echoPin

#define leftSonarSensorTrigPin 5
#define leftSonarSensorEchoPin 4

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

//Defining motor pins ; a1,a2 control LEFT_TIRE ; b1,b2 control RIGHT_TIRE

#define leftTireForward A1
#define leftTireBackward A0
#define rightTireForward A2
#define rightTireBackward A3

#define motorR1 12
#define motorR2 13

const int cDistance = 15; //13
int leftDist = 0;
int fDistance = 0;
int rightDist = 0;

double leftSonarSensorDistance = 0;
//Define time for events
unsigned long previousMillis_1 = 0; //Strore time for 1st event
unsigned long previousMillis_2 = 0; //stores time for 2nd event

//If sense infront turn right, if after millis still sense, turn left,while still sense turn left,after go forward
const long interval_1 = 800; //interval to be used for turning
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
  pinMode(sonarServoPin, OUTPUT);
  pinMode(gripperPin, OUTPUT);
  digitalWrite(sonarServoPin, LOW);
}

int number = 0;

//===== [LOOP] =====
void loop(){
  
  //rotateRight180();
     continuousForward();
     clearMotors();
     evadeCollision5();
//  rotateLeft90();
//  Serial.print("LOOPCOUNTER: ");
//  Serial.println(number);
//  continuousForward();
//  clearMotors();
//  evadeCollision5();  
//
// number+=1;
}

void rotateRight180(){
  
  int counter = 90;
  for(int i = 0; i < counter; i++){

    turnRight();
    delay(9);
  }
  clearMotors();
  //delay(300000000);
}

void rotateLeft90(){
  int counter = 90;
  for(int i = 0; i < counter; i++){

    turnLeft();
    delay(5);
  }
  clearMotors();
  delay(300000000);
}


void rotateRight90(){

 int counter = 90;
  for(int i = 0; i < counter; i++){

    turnRight();
    delay(5);
  }
  clearMotors();

}

void lookForward(){
  
  servo(sonarServoPin, sonarServoCenterPulse);
  delay(200);
}

void lookLeft(){
  
  servo(sonarServoPin, sonarServoMaxPulse);
  delay(80);
//   printLDistance();
}

void lookRight(){
  
  servo(sonarServoPin, sonarServoMinPulse);
  delay(80);
//    printRDistance();
}




/*
 * change the comparators in evadeColl5 to actual methods that return ldistance, rdistance
 * while LeftSonarSensorDistance < upperLimitDistance... run code below, if it's higher, turnLeft
  continuousForward()
  clearMotors();
  readLeft()-> lDistance updated
  readRight()->rDistance updated
    if lDistance > cDistance => turnLeft() for a certain milliseconds => moveForward;
    if rDistance > cDistance => turnRight() for a certain milliseconds => moveForward;
    if calcDistance in lDistance & rDistance < cDistance => moveBackward() for a certain milliseconds
     then readLeft()-> update lDistance
          readRight()-> update rDistance
      add a method that makes car turnLeft if leftSonarSensor_Distance > upperLimit_Distance, meaning if it senses a lot of space to the left() for a certain milliseconds => moveForward;
      if rDistance > cDistance => turnRight() for a certain milliseconds => moveForward;

      at every interval, scanDistance(){scanLeft, scanForward},if ldistance>cDistance,turnleft, if fdistance>cDistance, else turn right
  }
*/
void evadeCollision5(){
  
  lookLeft();
  //Serial.println("I looked left");
  if(leftDistance() > cDistance){
//     Serial.println("I read left");
     turnLeft(); 
    
    }else if(leftDistance() <= cDistance){
//      Serial.println("I read right");
      lookRight();
      
      if(rightDistance() > cDistance){
//       Serial.println("I turn right");
      turnRight(); 
      } else{ //Use millis
//      Serial.println("Backward occurs");
      //if(leftDist <= cDistance && rightDist <= cDistance)
//       Serial.println("Backward POGGERS1");
      moveBackward();
      delay(70);
      neoMoveBackward();
//      Serial.println("Backward POGGERS2");
     // delay(200);
//       Serial.println("I read left");
      lookLeft();
      if(leftDistance() > cDistance){
//          Serial.println("I turn left");
          turnLeft();
        }
     // delay(200);
    //  readRight();
     // delay(200);
    //  lookForward();
   //   Serial.println("Back to looking forward 2");
     // delay(200);
      else if(leftDistance() < cDistance){
//        Serial.println("I read right");
        lookRight();

        if(rightDistance() > cDistance){
//          Serial.println("I turn right");
          turnRight();
          }
      //neoTurnLeft();
      //Serial.println("I TurnLeft");
     // turnLeft(); 
      //delay(200);
      }
   //   else if(rDistance > cDistance){
      //neoTurnRight();
 //     Serial.println("I TurnLRight");
    //  turnRight();
      //delay(200);
     // }
      }  
    }
     
}

/*
checks left,if possible => turnLeft
checks forward, if possible => go forward
turnRight if both above cconditions fail
I can use either delay or microprocessor
*/
//void scan(){
//
//  readLeft();
//  if(lDistance > cDistance){
//        //turnLeft();
//        //neoTurnLeft();
//        delay(400);
//  }else{
//      readForward();
//      if(fDistance > cDistance){
//       //moveForward();
//       neoMoveForward();
//       delay(400);
//      } else{
//          //turnRight();
//          //lookRight();
//          neoTurnRight();
//          delay(400);
//        }
//    }
//    lookForward();
// //   Serial.println("Back to looking forward");
//}

void continuousForward(){
  lookForward();
  //printDistance();
  //Serial.println("I lookForward");
//  delay(200);
  while(calculateDistance() > cDistance){
     moveForward();
     neoMoveForward();
   //  Serial.println("calculateDistance() higher");
    }
   
    //Serial.println("cDistance higher");
}



//interrupt
void evadeCollision2(){//Movement logic using millis
 
 unsigned long currentMillis_1 =millis();
 printDistance();
 
  Serial.print("Time is:");
  Serial.println(currentMillis_1);
  
   moveForward();
   if(calculateDistance()<= cDistance){ //if distance<30 
   // printDistance();
    //clearMotors();
    turnRight();                //device turns right 
    
    if(currentMillis_1 - previousMillis_1 >= interval_1 ){  //if after 0,6s, run this
         //clearMotors();
          previousMillis_1 = millis();
          if(calculateDistance() <= cDistance){                       // and distance is still less than 60, turn left
            //clearMotors();
              while(calculateDistance() <= cDistance){        //while loop to ensure the device turns left completely
                 Serial.println("Stuck in while loop");
                turnLeft();
                printDistance();
//                if(currentMillis_1 - previousMillis_1 >= interval_2 ){  //  if stuck in turnLeft for an amount of time,goBackwards, after an amount of time,break
//                    moveBackward();
//                    delay(400);
//                    if(currentMillis_1 - previousMillis_1 > interval_3){
//                      break;
//                      }
//                  }
              }
              
            }
          // clearMotors();                                               //then moveForward()
          moveForward();
      }
    }
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
   moveForward();
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
          moveForward();
      }
    }
  }

void moveForward(){
  //if (calculateLeftSonarSensorDistance() <= 4.7 && calculateLeftSonarSensorDistance() > 1.8){
    analogWrite(leftTireForward,160);   // turns left tire forward
    analogWrite(rightTireForward,160);  //turns right tire forward
    analogWrite(leftTireBackward, 0);   //left tire backward 0
    analogWrite(rightTireBackward, 0);   //right tire backward 0
    neoMoveForward();
  //  }
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


int leftDistance(){
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
  int lDistance = duration * 0.034 / 2;

  leftDist = lDistance; 
  return lDistance;
}
void printLDistance(){ //Prints the distance calculated
// Prints the distance on the Serial Monitor
  //leftDistance();
  Serial.print("LDistance: ");
//  Serial.println(lDistance);
  Serial.println(leftDistance());
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
   leftSonarSensorDistance = duration * 0.034 / 2;

   double calculatedleftSonarSensor = leftSonarSensorDistance;

  return calculatedleftSonarSensor;
  
}

int rightDistance(){
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
  int rDistance = duration * 0.034 / 2;

  rightDist = rDistance;
  //rDistance = distance; 
  return rDistance;
}
void printRDistance(){ //Prints the distance calculated
// Prints the distance on the Serial Monitor
  //rightDistance();
  Serial.print("RDistance: ");
  //Serial.println(rDistance);
  Serial.println(rightDistance());
}



void printDistance(){ //Prints the distance calculated
// Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.println(calculateDistance());
}
