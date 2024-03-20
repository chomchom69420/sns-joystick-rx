#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "Wire.h"

#define I2C_DEV_ADDR 0x55

uint8_t broadcastAddress[] = {0xD8, 0xBC, 0x38, 0xE5, 0x1A, 0x28};

// Variable to store if sending data was successful
String success;

//Structure to store data
typedef struct struct_message {
    uint8_t j1PotX;
    uint8_t j2PotY;
} struct_message;

//Bot control variables
uint8_t throttle=0;
uint8_t steering=0;

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
  throttle = incoming_message.j2PotY;
  steering = incoming_message.j1PotX;
}

void onI2CRequest(){
  Wire.print(throttle);
  Serial.print(steering);
  Serial.println("onRequest");
}

void onI2CReceive(int len){
  Serial.printf("onReceive[%d]: ", len);
  while(Wire.available()){
    Serial.write(Wire.read());
  }
  Serial.println();
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  //Set-up I2C in slave mode
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);
  Wire.begin((uint8_t)I2C_DEV_ADDR);
 
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
 
void loop() {

}