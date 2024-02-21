#include "SimpleControlServo.h"
#define DEBUG

void setup() {
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
  print_to_serial();
#endif
  get_IMU();
  save_SD();
  delay(100);
}