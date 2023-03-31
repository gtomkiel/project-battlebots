long distance = 1000;

const int MOTOR_A1 = 9;//Left wheel backwards
const int MOTOR_A2 = 10;//Left wheel forward
const int MOTOR_B1 = 6;//Right wheel forward
const int MOTOR_B2 = 5;//Right wheel backwards

const int SENSOR_R1 = 3;  
const int SENSOR_R2 = 2; 


// defines pins numbers
const int trigPin = 8;
const int echoPin = 11;
// defines variables


void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  Serial.begin(9600); // Starts the serial communication
}

void turnBeforeObject(int powerA, int powerB)
{
  analogWrite(MOTOR_A2, powerA);
  analogWrite(MOTOR_B2, powerB);
}

void moveForward(int powerA,int powerB)
{
  analogWrite(MOTOR_A2,powerA);
  analogWrite(MOTOR_B1,powerB);
}

void moveBackward(int powerA,int powerB)
{

  analogWrite(MOTOR_A1,powerA);
  analogWrite(MOTOR_B2,powerB);
}

void idle(int powerA,int powerB)
{

  analogWrite(MOTOR_A1,powerA);
  analogWrite(MOTOR_A2,powerB);
  analogWrite(MOTOR_B1,powerB);
  analogWrite(MOTOR_B2,powerA);
}

void loop() {


  // moveForward(150, 170);
  // moveBackward(0,0);

  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.println(distance);
  delay(500);



  if (distance < 16)
  {
    idle(0, 0);
    delay(1000);
    turnBeforeObject(250, 250);
    delay(250);
    moveForward(120, 250);
    moveBackward(0,0);
    delay(2350);
    idle(0,0);
    delay(500);
    turnBeforeObject(250, 250);
    delay(105);
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

