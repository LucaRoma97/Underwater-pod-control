#include "SimpleControlServo.h"
#define DEBUG

void setup() {
  #ifdef DEBUG
  Serial.begin(9600);
  #endif 
  initialize_outputs(); 
  initialize_depth();
  initialize_SD(); // Start SD
  initialize_IMU();
}

void loop() {
  read_RC();
  get_IMU();
  get_depth();

  RC_2_control(); 

  pitch_control();
  depth_control();
  yaw_control();

  //get_pumps_out();
  write_outputs();
#ifdef DEBUG
  print_to_serial();
#endif
  save_SD();
  delay(100);
}