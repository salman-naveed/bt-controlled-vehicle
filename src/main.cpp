#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Servo.h>

/*
   Arduino Robot Vehicle Wireless Control using the HC-05 Bluetooth and custom-build Android app

             == SLAVE DEVICE - Arduino Robot Vehicle OBC ==

   by Salman Naveed
*/

#define enA 5
#define in1 3
#define in2 4
#define enB 6
#define in3 7
#define in4 8

#define servoPin 11

int xAxis, yAxis;
int  x = 0;
int  y = 0;

int motorSpeedA = 0;
int motorSpeedB = 0;

SoftwareSerial BTConnect(2, 13); // RX, TX
int dataIn[5] {0,0,0,0}; //array to store all the information. 255,button,X,Y.
int in_byte = 0;
int array_index = 0;

Servo steerServo;  // create servo object to control a servo
int pos = 0;    // variable to store the servo position
const int steerMidPoint = 30;

void steering(int steerDeflection)
{
    steerDeflection = steerDeflection / 4;
    int steerAngle = steerMidPoint + map(steerDeflection, 0, 255, 25, -25);
    steerServo.write(steerAngle);
    delay(15);
}

void drive(int y_Axis)
{
  if (y_Axis < 470) {
    // Set Motor A backward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    // Set Motor B backward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    // Convert the declining Y-axis readings for going backward from 470 to 0 into 0 to 255 value for the PWM signal for increasing the motor speed
    motorSpeedA = map(y_Axis, 470, 0, 0, 255);
    motorSpeedB = map(y_Axis, 470, 0, 0, 255);
       
  }

  else if (y_Axis > 550) {
    // Set Motor A forward
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    // Set Motor B forward
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    // Convert the increasing Y-axis readings for going forward from 550 to 1023 into 0 to 255 value for the PWM signal for increasing the motor speed
    motorSpeedA = map(y_Axis, 550, 1023, 0, 255);
    motorSpeedB = map(y_Axis, 550, 1023, 0, 255);
  }

  // If joystick stays in middle the motors are not moving
  else {
    motorSpeedA = 0;
    motorSpeedB = 0;
  }    
  
  // Prevent buzzing at low speeds (Adjust according to your motors. My motors couldn't start moving if PWM value was below value of 70)
  if (motorSpeedA < 70) {
    motorSpeedA = 0;
  }
  if (motorSpeedB < 70) {
    motorSpeedB = 0;
  }
  analogWrite(enA, motorSpeedA); // Send PWM signal to motor A
  analogWrite(enB, motorSpeedB); // Send PWM signal to motor B
}


void setup() {
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  steerServo.attach(servoPin);
  steerServo.write(steerMidPoint);

  //Serial.begin(9600); // Default communication rate of the Bluetooth module
  BTConnect.begin(9600);
}


void loop() {
  // Default value - no movement when the Joystick stays in the center
  xAxis = 511;
  yAxis = 511;

  // Read the incoming data from the Smartphone Android App
  if(BTConnect.available() > 0) {
    in_byte = BTConnect.read(); //store in byte into a variable
    if (in_byte == (255)) { // if the variable is 0 set the array inxed to 0. this will make sure that every number goes into the correct index
    array_index = 0;
      }
  dataIn[array_index] = in_byte;  //store number into array
  array_index = array_index +1;
    }
    x = dataIn[1];    
    y = dataIn[2]; 
  
   // Makes sure we receive correct values
  if (x > 0 & x < 250) {
    xAxis = map(x, 0, 250, 0, 1023); // Convert the smartphone X and Y values to 0 - 1023 range, suitable for the motor control code below
  }
  if (y > 0 & y < 250) {
    yAxis = map(y, 0, 250, 1023, 0);
  }
  //Serial.println(yAxis);
  drive(yAxis);
  steering(xAxis);
   
}