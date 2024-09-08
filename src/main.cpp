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

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <ctype.h>
#include <stdlib.h>

#define MOTORS           1
#define JOYSTICK         0

//Motor direction defines
#define MOTOR_F_FWD 0
#define MOTOR_R_FWD 0

#define MAX_INPUT_LENGTH 32
#define RL_C1     2 // Port D
#define RR_C1     A0 // Port B
#define PWM1      13
#define PWM2      12
#define DIR_PIN   14
#define FRONT_DIR1 7
#define FRONT_DIR2 8
#define REAR_DIR1  A5
#define REAR_DIR2  A4

#define CE  10
#define CSN 4

int speed_pwm = 0;
bool isPwm=true;
float left_pwm = speed_pwm;
float right_pwm = left_pwm+1;

int motorSpeed_FL = 0;
int motorSpeed_FR = 0;
int motorSpeed_RL = 0;
int motorSpeed_RR = 0;

char inputBuffer[MAX_INPUT_LENGTH];
bool isRunning = false;

//Variables for remote control
uint8_t steering=70;
uint8_t throttle=85;

//Motor speed variables
int motorSpeedA = 0;
int motorSpeedB = 0;

uint8_t broadcastAddress[] = {0xD8, 0xBC, 0x38, 0xE5, 0x1A, 0x28};

// Variable to store if sending data was successful
String success;

//Structure to store data
typedef struct struct_message {
    uint8_t j1PotX;
    uint8_t j2PotY;
} struct_message;

uint32_t i = 0;

//struct for incoming messages
struct_message incoming_message;

//Globals
esp_now_peer_info_t peerInfo;

// Callback when data is sent
void on_esp_now_data_sent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status==0){
    success = "Delivery Success";
  }
  else{
    success = "Delivery Fail";
  }
}

// Callback when data is received
void on_esp_now_data_recv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incoming_message, incomingData, sizeof(incoming_message));
  // Serial.println("Message received");
  steering = incoming_message.j2PotY;
  throttle = incoming_message.j1PotX;
  // Serial.print("Throttle: ");
  // Serial.print(throttle);
  // Serial.print(" Steering: ");
  // Serial.print(steering);
  // Serial.println();
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
	while (!Serial && millis() < 5000);
  delay(500);
  memset(inputBuffer,0,MAX_INPUT_LENGTH);

  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  // pinMode(FRONT_DIR1, OUTPUT);
  // pinMode(FRONT_DIR2, OUTPUT);
  // pinMode(REAR_DIR1, OUTPUT);
  // pinMode(REAR_DIR2, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  // digitalWrite(FRONT_DIR1, MOTOR_F_FWD);
  // digitalWrite(FRONT_DIR2, MOTOR_F_FWD);
  // digitalWrite(REAR_DIR1,  MOTOR_R_FWD);
  // digitalWrite(REAR_DIR2,  MOTOR_R_FWD);
  digitalWrite(DIR_PIN, MOTOR_F_FWD);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(on_esp_now_data_sent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  else {
    Serial.println("Added peer successfully");
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(on_esp_now_data_recv);
}
 
void loop()
{  
  static int inputPos = 0;
  if (Serial.available() > 0) {
    char received = Serial.read();
    // End of command
    if (received == '\n') {
      inputBuffer[inputPos] = '\0'; // Null-terminate the input string
      // Check for START command
      if (strncmp(inputBuffer, "START", 5) == 0) {
        Serial.println("START");
        isRunning = true;
        int pwmValue = atoi(&inputBuffer[5]); // Convert to integer starting from the 5th character
        speed_pwm = pwmValue;
        left_pwm = speed_pwm;
        right_pwm = left_pwm+1;

        if (pwmValue > 255) {
          // Reverse direction and adjust PWM value if necessary
          digitalWrite(DIR_PIN, !MOTOR_F_FWD);
          pwmValue-=255;
          speed_pwm = pwmValue;
          left_pwm = speed_pwm;
          right_pwm = left_pwm+1;
        }
      } 
      // Check for STOP command
      else if (strcmp(inputBuffer, "STOP") == 0) {
        Serial.println("STOP");
        int derate=20;
        while(motorSpeedA>0 ||  motorSpeedB>0){
          motorSpeedA=max(0,motorSpeedA-derate);
          motorSpeedB=max(0,motorSpeedB-derate);
          #ifdef MOTORS
          analogWrite(PWM1, motorSpeedA);
          analogWrite(PWM2, motorSpeedB);
          #endif
          delay(500);
        }
        isRunning = false;
         // Reset PWM output
      } 
      // Check for PWM command
      else if (strncmp(inputBuffer, "PWM", 3) == 0 && isPwm==true) {
        // Attempt to read the PWM value from the command
        isPwm=false;
      }
      // Reset for the next command
      inputPos = 0;
      memset(inputBuffer, 0, MAX_INPUT_LENGTH);
    } 
    else if (received != '\r') { // Ignore carriage return
      // Store the received character into the buffer
      if (inputPos < MAX_INPUT_LENGTH - 1) {
        inputBuffer[inputPos++] = received;
      }
    }
  }
 if (isRunning==true){

  //------------------- joystick code ---------------------------------------------------
  #if JOYSTICK
  Serial.println(throttle);

  // Throttle used for forward and backward control
  // Joystick values: 0 to 255; down = 0; middle = 127; up = 255
  if (throttle <= 70) {
    //Set all 4 motors backward
    digitalWrite(DIR_PIN, !MOTOR_F_FWD);
    // Convert the declining throttle readings for going backward from 110 to 0 into 0 to 255 value for the PWM signal for increasing the motor speed
    motorSpeedA = map(throttle, 70, 0, 0, 255);
    motorSpeedB = map(throttle, 70, 0, 0, 255);
  }
  else if (throttle >= 120) {
    //Set all 4 motors forward 
    digitalWrite(DIR_PIN, MOTOR_F_FWD);
    // Convert the increasing throttle readings for going forward from 140 to 255 into 0 to 255 value for the PWM signal for increasing the motor speed
    motorSpeedA = map(throttle, 120, 255, 0, 255);
    motorSpeedB = map(throttle, 120, 255, 0, 255);
  }
  // If joystick stays in middle the motors are not moving
  else {
    motorSpeedA = 0;
    motorSpeedB = 0;
  }
  
  // // Steering used for left and right control
  // if (steering <= 65) {
  //   // Convert the declining steering readings from 140 to 255 into increasing 0 to 255 value
  //   int xMapped = map(steering, 65, 0, 0, 255);
  //   // Move to left - decrease left motor speed, increase right motor speed
  //   motorSpeedA = motorSpeedA - xMapped;
  //   motorSpeedB = motorSpeedB + xMapped;
  //   // Confine the range from 0 to 255
  //   if (motorSpeedA < 0) {
  //     motorSpeedA = 0;
  //   }
  //   if (motorSpeedB > 255) {
  //     motorSpeedB = 255;
  //   }
  // }
  // if (steering >= 75) {
  //   // Convert the increasing steering readings from 110 to 0 into 0 to 255 value
  //   int xMapped = map(steering, 75, 255, 0, 255);
  //   // Move right - decrease right motor speed, increase left motor speed
  //   motorSpeedA = motorSpeedA + xMapped;
  //   motorSpeedB = motorSpeedB - xMapped;
  //   // Confine the range from 0 to 255
  //   if (motorSpeedA > 255) {
  //     motorSpeedA = 255;
  //   }
  //   if (motorSpeedB < 0) {
  //     motorSpeedB = 0;
  //   }
  // }
  #endif

  #if !JOYSTICK
  motorSpeedA = round(left_pwm);
  motorSpeedB = round(right_pwm);
  #endif

  analogWrite(PWM1, motorSpeedA);
  analogWrite(PWM2, motorSpeedB);
  // Serial.println(motorSpeedA);
  // Serial.println(motorSpeedB);
 }
}