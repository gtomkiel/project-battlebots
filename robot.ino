#include <Adafruit_NeoPixel.h>

#define PIN        6 // Pin for LEDs
#define NUMPIXELS  4 // Number of LEDs

// Pins connected to motor controller
const int motorLeftFWD = 13;
const int motorLeftBWD = 12;
const int motorRightFWD = 11;
const int motorRightBWD = 10;

const int rotor1 = A6;
const int rotor2 = A7;  

const int motorR1 = 3;
const int motorR2 = 2;

#define trigPin 5
#define echoPin 4

long duration;
int distance;

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// Function to operate LEDs when robot is idle
// You need to specify mode of the lights (currently 1 or 2)
void lightsIdle(int mode) {
  switch(mode){
    case 1:
      for(int i=0; i<NUMPIXELS; i++){
        delay(5);

        pixels.clear();
        pixels.show();

        delay(2000);

        for(int pin=0; pin<NUMPIXELS; pin++){
          pixels.setPixelColor(pin, pixels.Color(0, 150, 0));
        }

        pixels.show();
      }
      break;

    case 2:
      for(int i=0; i<NUMPIXELS; i++){
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

// guess what
int getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;

  return distance;

  delay(50);
}

void motorForward(){
  delay(1000);
  pixels.clear();

  if(getDistance() > 50) {
    for(int pin=0; pin<NUMPIXELS; pin++){
    pixels.setPixelColor(pin, pixels.Color(255, 255, 255));
    }
    pixels.show();

    analogWrite(motorLeftFWD, 200);
    analogWrite(motorRightFWD, 200);
  }

  pixels.clear();
  for(int pin=0; pin<NUMPIXELS; pin++){
    pixels.setPixelColor(pin, pixels.Color(0, 255, 0));
  }
  pixels.show();

  analogWrite(motorLeftFWD, 0);
  analogWrite(motorRightFWD, 0);
}

// Sensors
 int sensor_one = A5;
 int sensor_two = A4;
 int sensor_three = A3;
 int sensor_four = A2;
 int sensor_five = A1;
 int sensor_six = A0;

void object(){
  int distance = getDistance();

  if(distance < 15){
    pixels.clear();
    for(int pin=0; pin<NUMPIXELS; pin++){
      pixels.setPixelColor(pin, pixels.Color(0, 150, 0));
    }
    pixels.show();

    analogWrite(motorLeftFWD, 0);
    analogWrite(motorRightFWD, 0);
  } else {
    pixels.clear();
    for(int pin=0; pin<NUMPIXELS; pin++){
      pixels.setPixelColor(pin, pixels.Color(255, 255, 255));
    }
    pixels.show();

    analogWrite(motorLeftFWD, 200);
    analogWrite(motorRightFWD, 200);
  }
}

void mazeForward()
{
  int sensorFour = analogRead(sensor_four);
  int sensorFive = analogRead(sensor_five);

  int sensorOne = analogRead(sensor_one);
  int sensorTwo = analogRead(sensor_two);
  int sensorThree = analogRead(sensor_three);
  int sensorSix = analogRead(sensor_six);


  if(sensorThree > 600 && sensorFour > 600)
  {
    analogWrite(motorLeftFWD, 200);
    analogWrite(motorRightFWD, 200);  
  }
  else if(sensorOne > 600 && sensorTwo > 600)
  {
    analogWrite(motorLeftFWD, 0);
    analogWrite(motorRightFWD, 200); 
  }
  else if(sensorFive > 600 && sensorSix > 600)
  {
    analogWrite(motorLeftFWD, 200);
    analogWrite(motorRightFWD, 0); 
  }
  else if(sensorThree < 600 && sensorFour < 600)
  {
    analogWrite(motorLeftFWD, 150);
    analogWrite(motorRightFWD, 0);

    /*
    if(sensorFive > 600 && sensorSix > 600)
    {
      analogWrite(motorLeftFWD, 200);
      analogWrite(motorRightFWD, 0); 
    }

    if(sensorOne > 600 && sensorTwo > 600)
    {
      analogWrite(motorLeftFWD, 0);
      analogWrite(motorRightFWD, 200); 
    }
    */
  }
}

void testTurn()
{
  analogWrite(motorLeftFWD, 200);
  analogWrite(motorRightBWD, 200);
  delay(500);
  analogWrite(motorLeftFWD, 0);
  analogWrite(motorRightBWD, 0);
  delay(500);
  analogWrite(motorLeftBWD, 200);
  analogWrite(motorRightFWD, 200);
  delay(500);
  analogWrite(motorLeftBWD, 0);
  analogWrite(motorRightFWD, 0);
  delay(500);
}

void lineDebug()
{
  Serial.print(sensor_one);
  Serial.print(" ");
  Serial.print(sensor_two);
  Serial.print(" ");
  Serial.print(sensor_three);
  Serial.print(" ");
  Serial.print(sensor_five);
  Serial.print(" ");
  Serial.println(sensor_six);
}

void setup() {
  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.clear();
  pixels.show();

  pinMode(motorLeftFWD, OUTPUT);
  pinMode(motorRightFWD, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(sensor_one, INPUT);
  pinMode(sensor_two, INPUT);
  pinMode(sensor_three, INPUT);
  pinMode(sensor_four, INPUT);
  pinMode(sensor_five, INPUT);
  pinMode(sensor_six, INPUT);

  
  //Begin Serial communication at a baudrate of 9600:
  Serial.begin(9600);
}

void loop() {
  lineDebug();
}