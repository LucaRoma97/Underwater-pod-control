#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <SD.h>
#include "SimpleControlServo.h"

#define DEBUG

void setup() {
  // put your setup code here, to run once:
  #ifdef DEBUG
  Serial.begin(9600);
  #endif 
  initialize_outputs(); 
  initialize_SD(); // Start SD
  initialize_IMU();
}

void loop() {

  read_IMU();
  read_RC();
  RC_2_control(); 
  get_pumps_out();
  write_outputs();

  
#ifdef DEBUG
  Serial.println(forwardRate);
  Serial.println(chPitch_value);
  Serial.println(chBuoyancy_value);
  Serial.println(chThrottle_value);
  Serial.println(chTurn_value);
  Serial.println(Windingval);
  Serial.println(chGrasping_value);
#endif
  get_IMU();
  save_SD();
  delay(100);
}
