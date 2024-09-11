/*
DIR1 --> Green 
DIR2 --> Orange
PWM1 --> Blue
PWM2 --> Purple
*/

/*
Motor colour coding 
M-  --> Black
M+  --> Red
Vcc --> Violet
Gnd --> White
ChA --> Green
*/

/*
I2C colour coding
SDA --> Brown 
SCL --> Yellow
*/
#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <DabbleESP32.h>
#include <Arduino.h>
#include <ctype.h>
#include <stdlib.h>

//Motor direction defines
#define MOTOR_F_FWD 0
#define MOTOR_R_FWD 0

#define PWM1      13
#define PWM2      12
#define DIR_PIN   14

//Variables for remote control
float steering=0;
float throttle=0;

//Motor speed variables
int motorSpeedL = 0;
int motorSpeedR = 0;
 
void setup() {
  Serial.begin(115200);
	while (!Serial && millis() < 5000);
  delay(500);
  Dabble.begin("MyEsp32");

  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, MOTOR_F_FWD);;
}
 
void loop()
{  
  //------------------- joystick code ---------------------------------------------------
  Dabble.processInput();  
  float x = GamePad.getXaxisData();
  float y = GamePad.getYaxisData();

  /*Joystick values
  * x --> -6 to 7 increments of 1 --> PWM : 0 to 255
  * y --> -6 to 7 increments of 1 --> Steering: -255 to 255
  */
  throttle = y;
  steering = x;

  // Throttle used for forward and backward control
  // Joystick values:
  if (throttle < 0) {
    //Set all 4 motors backward
    digitalWrite(DIR_PIN, !MOTOR_F_FWD);
    // Convert the declining throttle readings for going backward from 110 to 0 into 0 to 255 value for the PWM signal for increasing the motor speed
    motorSpeedL = map(throttle, 0, -6, 0, 255);
    motorSpeedR = map(throttle, 0, -6, 0, 255);
  }
  else if (throttle > 0) {
    //Set all 4 motors forward 
    digitalWrite(DIR_PIN, MOTOR_F_FWD);
    // Convert the increasing throttle readings for going forward from 140 to 255 into 0 to 255 value for the PWM signal for increasing the motor speed
    motorSpeedL = map(throttle, 0, 7, 0, 255);
    motorSpeedR = map(throttle, 0, 7, 0, 255);
  }
  // If joystick stays in middle the motors are not moving
  else {
    motorSpeedL = 0;
    motorSpeedR = 0;
  }

  // Steering used for left and right control
  if (steering <= 0) {
    // Convert the declining steering readings from 140 to 255 into increasing 0 to 255 value
    int xMapped = map(steering, 0, -6, 0, 255);
    // Move to left - decrease left motor speed, increase right motor speed
    motorSpeedL = motorSpeedL - xMapped;
    motorSpeedR = motorSpeedR + xMapped;
    // Confine the range from 0 to 255
    if (motorSpeedL < 0) {
      motorSpeedL = 0;
    }
    if (motorSpeedR > 255) {
      motorSpeedR = 255;
    }
  }
  if (steering > 0) {
    // Convert the increasing steering readings from 110 to 0 into 0 to 255 value
    int xMapped = map(steering, 0, 7, 0, 255);
    // Move right - decrease right motor speed, increase left motor speed
    motorSpeedL = motorSpeedL + xMapped;
    motorSpeedR = motorSpeedR - xMapped;
    // Confine the range from 0 to 255
    if (motorSpeedL > 255) {
      motorSpeedL = 255;
    }
    if (motorSpeedR < 0) {
      motorSpeedR = 0;
    }
  }
  analogWrite(PWM1, motorSpeedL);
  analogWrite(PWM2, motorSpeedR);
}