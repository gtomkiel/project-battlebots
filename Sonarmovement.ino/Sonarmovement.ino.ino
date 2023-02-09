/** THIS PROGRAM IS FOR RACE CAR EVADING COLLISION
   HC-SR04 has a range 2cm to 400cm
The sensor is composed of two ultrasonic transducers. One is transmitter which outputs ultrasonic sound pulses and the other is receiver which listens for reflected waves. 
It’s basically a SONAR which is used in submarines for detecting underwater objects.

Trig 10 -> Green wire sends  ultrasonic wave
Echo 9  -> Yellow wire listens for reflected signal
Considering the travel time and the speed of the sound you can calculate the distance.

===================[CURRENT FAULTS]===================
evadeCollision2() contains logic to move backwards if turnRight() then turnLeft() after a couple of seconds doesnt work, then moveBackward() should occur for a time interval, 
then moveForward or turnLeft should occur for a couple seconds, then moveForward
All this occurs if sonarSensor senses a distance<collisionDistance(cDistance)


===================[FUTURE TASKS]===================
  Learn how to use the gripper(read libraries & create own functionalities3)
  Learn how to use line sensor(works for tape, darker tape,higher value & vice-versa for lighter tape) 
  Then research how to control the speed of the tires-> Figure out how to control speed using pulses/second ;
  research PWM(6 of them)
  Then bluetooth configuration(Search bluetooth 2 way connector)
  Learn how to use bluettoth to activate robot(relay race related)
*/


//Initialising use of Ardafruit

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

//Initialising Servo
#include <Servo.h>

Servo gripper; //servo object named gripper

/*
 Pin position for each Line sensor's wire
white A4
grey  A5
purple A6
blue  A7
green  D3
yellow D5
orange  D9
brown   D10
*/

// define pin numbers
const int trigPin = 7;
const int echoPin = 6;

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

const int a1 = A0;
const int a2 = A1;
const int b1 = A2;
const int b2 = A3;





/* MOtor setuo using digitalWrite()
  #define a1 6 //blue a0

  #define a2 7 //black a1

  #define b1 4 //orange a2

  #define b2 5 //yellow a3
*/
const int cDistance = 45;

//Define time for events
unsigned long previousMillis_1 = 0; //Strore time for 1st event
unsigned long previousMillis_2 = 0; //stores time for 2nd event

//If sense infront turn right, if after millis still sense, turn left,while still sense turn left,after go forward
const long interval_1 = 500; //interval to be used for turning
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

  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  Serial.begin(9600); // Starts the serial communication

  //Setting up motor pins
  pinMode (a1, OUTPUT);
  pinMode (a2, OUTPUT);
  pinMode (b1, OUTPUT);
  pinMode (b2, OUTPUT);

  gripper.attach(2);        //Initialising Gripper pin
}
//const int pina1 = ;
// analogWrite(pina1, 255)


//===== [LOOP] =====
void loop(){

  // evadeCollision();
  //moveForward();
}

/*
  Set interval to ensure gripper isOpen before movement
*/
void gripperOpen(){
  
    int upperBound = 124;               //Best gripperOpen angle found from experimenting
    gripper.write(upperBound);  
  }

void gripperClose(){
  
    int lowerBound = 45;                //Best gripperClose angle found from experimenting
    gripper.write(lowerBound);
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

void evadeCollision2(){//Movement logic using millis
 
 unsigned long currentMillis_1 =millis();
 printDistance();
 
  Serial.print("Time is:");
  Serial.println(currentMillis_1);
  
   moveForward();
   if(calculateDistance()< cDistance){ //if distance<30 
   // printDistance();
    clearMotors();
    turnRight();                //device turns right 
    
    if(currentMillis_1 - previousMillis_1 >= interval_1 ){  //if after 0,6s, run this
         clearMotors();
          previousMillis_1 = millis();
          if(calculateDistance()< cDistance){                       // and distance is still less than 60, turn left
            clearMotors();
              while(calculateDistance() < cDistance){        //while loop to ensure the device turns left completely
                 Serial.println("Stuck in while loop");
                turnLeft();
                printDistance();
                if(currentMillis_1 - previousMillis_1 >= interval_2 ){  //  if stuck in turnLeft for an amount of time,goBackwards, after an amount of time,break
                    moveBackward();
                    if(currentMillis_1 - previousMillis_1 > interval_3){
                      break;
                      }
                  }
              }
              clearMotors();
            }
           clearMotors();                                               //then moveForward()
          moveForward();
      }
    }
  }

void evadeCollision(){//Movement logic using millis
 
 unsigned long currentMillis_1 =millis();
 printDistance();
 
  Serial.print("Time is:");
  Serial.println(currentMillis_1);
  
   moveForward();
   if(calculateDistance()< cDistance){ //if distance<30 
   // printDistance();
    clearMotors();
    turnRight();                //device turns right 
    
    if(currentMillis_1 - previousMillis_1 >= interval_1 ){  //if after 0,5s
         
          previousMillis_1 = millis();
          if(calculateDistance()< cDistance){                       // and distance is still less than 60, turn left
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
  analogWrite(a2,255); // turns left tire forwards
  analogWrite(b1,255);   //turns right tire forwards

   neoMoveForward();
  }

void turnRight(){ //This moves the my left tire forwards and stops the right tire from rotating
  analogWrite(a2,225); // turns left tire forwards
  analogWrite(b2,255);   // truerns right tire backwards

  neoTurnRight();
}

void turnLeft(){ //This moves the my left tire backwards and stops the right tire from rotating
  analogWrite(a1,225); //turns left tire backwards
  analogWrite(b1,255);   //turns right tire forwards

  neoTurnLeft();
}

void moveBackward(){ //Moves both tires backwards
  analogWrite(b2,255);   // truerns right tire backwards
  analogWrite(a1,255); //turns left tire backwards
  neoMoveBackward();
}

void clearMotors(){ //Stops motor by setting both tires to the same signal
  analogWrite(b2,0);   // truerns right tire 0
  analogWrite(a1,0); //turns left tire 0
  analogWrite(b1,0);   // truerns right tire 0
  analogWrite(a2,0); //turns left tire 0
}

void neoMoveForward(){
  int n1 = 0;       //backleft
  int n2 = 1;       //backright
  int n3 =2;        //topright
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
  int n3 =2;        //topright
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
  int n3 =2;        //topright
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
  int n3 =2;        //topright
  int n4 = 3;       //topleft 
   strip.setPixelColor(n4,0, 0,0); //means topleft is set to green (g,r,b)
   strip.setPixelColor(n3, 0, 0 , 0); //means topright is set to green (g,r,b)
   strip.setPixelColor(n2, 0, 255 , 0); //means backright is set to green (g,r,b)
   strip.setPixelColor(n1, 0, 255 , 0); //means backleft is set to green (g,r,b)
   strip.show();
}

int calculateDistance(){ //Calculates distance to the sonar sensor
// Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);

  // Calculating the distance
  int distance = duration * 0.034 / 2;

  return distance;
}

void printDistance(){ //Prints the distance calculated
// Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.println(calculateDistance());
}
