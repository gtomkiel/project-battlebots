/** THIS PROGRAM IS FOR RACE CAR FOLLOWING LINE
  QTR-8RC sensor is made up of 8 led sensors connected to a board

Pin position for each sensor's wire
white A4
grey  A5
purple A6
blue  A7
green  D3
yellow D5
orange  D11
brown   D12

Considering the travel time and the speed of the sound you can calculate the distance.

===================[CURRENT FAULTS]===================
If sensor within black tape threshold => value set to 1
if not black tape threshold =>     value is 0
if 00011100 -> could be moveForwardand right alittle bit like 80 degrees forward
if 11100000 -> turnLeft

Add a speaker for Wall-E noises
improve motor output from 50% => something higher

===================[FUTURE TASKS]===================
  Learn how to use line sensor(works for tape, darker tape,higher value & vice-versa for lighter tape) 
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


//Defining motor pins ; a1,a2 control LEFT_TIRE ; b1,b2 control RIGHT_TIRE

const int a1 = A0;
const int a2 = A1;
const int b1 = A2;
const int b2 = A3;


//Define time for events
unsigned long previousMillis_1 = 0; //Strore time for 1st event
unsigned long previousMillis_2 = 0; //stores time for 2nd event



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
  
  Serial.begin(9600); // Starts the serial communication

  //Setting up motor pins
  pinMode (a1, OUTPUT);
  pinMode (a2, OUTPUT);
  pinMode (b1, OUTPUT);
  pinMode (b2, OUTPUT);

  gripper.attach(2);        //Initialising Gripper pin
  sonarServo.attach(13);
}

//=====[LOOP]======
void loop(){
   //moveForward();
  // evadeCollision();
  //moveForward();
  //gripperOpen();
 // printDistance();
 //scan();
}

//======[GRIPPER]======
void gripperOpen(){
  
    int upperBound = 124;               //Best gripperOpen angle found from experimenting
    gripper.write(upperBound);  
  }

void gripperClose(){
  
    int lowerBound = 45;                //Best gripperClose angle found from experimenting
    gripper.write(lowerBound);
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
