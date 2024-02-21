#include "Arduino_BMI270_BMM150.h"
#define DEBUG
#include <PPMReader.h>
#define DATA_ON_SD 26
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <SD.h>

#include "Adafruit_AHRS_Mahony.h"
#include "Adafruit_AHRS_Madgwick.h"
#include <Adafruit_AHRS_NXPFusion.h>
#include <Arduino_BMI270_BMM150.h>
#include <Wire.h>


/// IMU ///
class MyBoschSensor : public BoschSensorClass {

public:
  MyBoschSensor(TwoWire& wire = Wire)
    : BoschSensorClass(wire){};

protected:
  virtual int8_t configure_sensor(struct bmi2_dev* dev) {
    int8_t rslt;
    uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_GYRO };

    struct bmi2_int_pin_config int_pin_cfg;
    int_pin_cfg.pin_type = BMI2_INT1;
    int_pin_cfg.int_latch = BMI2_INT_NON_LATCH;
    int_pin_cfg.pin_cfg[0].lvl = BMI2_INT_ACTIVE_HIGH;
    int_pin_cfg.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
    int_pin_cfg.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
    int_pin_cfg.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;

    struct bmi2_sens_config sens_cfg[2];
    sens_cfg[0].type = BMI2_ACCEL;
    sens_cfg[0].cfg.acc.bwp = BMI2_ACC_OSR2_AVG2;
    sens_cfg[0].cfg.acc.odr = BMI2_ACC_ODR_100HZ;
    sens_cfg[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    sens_cfg[0].cfg.acc.range = BMI2_ACC_RANGE_4G;
    sens_cfg[1].type = BMI2_GYRO;
    sens_cfg[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    sens_cfg[1].cfg.gyr.bwp = BMI2_GYR_OSR2_MODE;
    sens_cfg[1].cfg.gyr.odr = BMI2_GYR_ODR_100HZ;
    sens_cfg[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
    sens_cfg[1].cfg.gyr.ois_range = BMI2_GYR_OIS_2000;

    rslt = bmi2_set_int_pin_config(&int_pin_cfg, dev);
    if (rslt != BMI2_OK)
      return rslt;

    rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, dev);
    if (rslt != BMI2_OK)
      return rslt;

    rslt = bmi2_set_sensor_config(sens_cfg, 2, dev);
    if (rslt != BMI2_OK)
      return rslt;

    rslt = bmi2_sensor_enable(sens_list, 2, dev);
    if (rslt != BMI2_OK)
      return rslt;

    return rslt;
  }
};

MyBoschSensor myIMU(Wire1);

float mag_offsets[3] = { -40.42F, 13.36F, 0.91F };

// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = { { 1.022, 0.003, -0.02 },
                                    { 0.003, 0.973, -0.012 },
                                    { -0.02, -0.012, 1.006 } };

float mag_field_strength = 45.44F;

// Offsets applied to compensate for gyro zero-drift error for x/y/z
// Raw values converted to rad/s based on 250dps sensitiviy (1 lsb = 0.00875 rad/s)
float rawToDPS = 0.00875F;
float dpsToRad = 0.017453293F;
float gyro_zero_offsets[3] = { -0.9155F, -0.1831F, -0.1526F };
 

// Mahony is lighter weight as a filter and should be used
// on slower systems
//Adafruit_Mahony filter;
Adafruit_Madgwick filter;
//Adafruit_NXPSensorFusion filter;

float accelX, accelY, accelZ,          // units m/s/s i.e. accelZ if often 9.8 (gravity)
  gyroX, gyroY, gyroZ,                 // units dps (degrees per second)
  gyroDriftX, gyroDriftY, gyroDriftZ,  // units dps
  magX, magY, magZ, mag_x, mag_y;

long lastInterval, lastTime, a;

void readIMU() {
   myIMU.readAcceleration(accelX, accelY, accelZ);
   myIMU.readGyroscope(gyroX, gyroY, gyroZ);
   myIMU.readMagneticField(magX, magY, magZ);
  };
void calibrateIMU(int delayMillis, int calibrationMillis) {

  int calibrationCount = 0;

  delay(delayMillis);  // to avoid shakes after pressing reset button
  

  float sumX, sumY, sumZ;
  int startTime = millis();
  while (millis() < startTime + calibrationMillis) {
    readIMU();
    // in an ideal world gyroX/Y/Z == 0, anything higher or lower represents drift
    sumX += gyroX;
    sumY += gyroY;
    sumZ += gyroZ;
    calibrationCount++;
  }

  if (calibrationCount == 0) {
    Serial.println("Failed to calibrate");
  }

  gyroDriftX = sumX / calibrationCount;
  gyroDriftY = sumY / calibrationCount;
  gyroDriftZ = sumZ / calibrationCount;
}

/// IMITIALIZATIONS ///

int ch_throttle = 3;
int ch_turn = 4;
int ch_pitch = 1;
int ch_buoyancy = 2;
int ch_grasping = 6;
int ch_winding = 5;

volatile int chThrottle_value = 1000;//chThrottle_
volatile int chPitch_value = 1500;//chPitch_
volatile int chBuoyancy_value = 1500;//chBuoyancy_
volatile int chTurn_value = 1500;
volatile int chGrasping_value = 1500;
volatile int chWinding_value = 1500;

int chPitch_max = 1916;
int chBuoyancy_max = 1802;
int chThrottle_max = 1921;
int chTurn_max = 1936;

int chPitch_min = 1098;
int chBuoyancy_min = 971;
int chThrottle_min = 1000;
int chTurn_min = 1117;

int degreesX = 0;
int degreesY = 0;
int degreesZ = 0;

int ServoPos;
int PumpLeftRate;
int PumpRightRate;
int PumpUpRate;
int PumpDownRate;
int TurnRate;
int PitchRate;
int forwardRate;
int Windingval;

int ServoPin = 10;
int PumpLeftPin = 5;
int PumpRightPin = 3;
int PumpUpPin = 7;
int PumpDownPin = 9;
int GrasperPin = 3;
int WindingPin = 8;

byte interruptPin = 6;

char fileName[13];
static int fileCounter = 0;
const int chipSelect = A3;

byte channelAmount = 6;
PPMReader ppm(interruptPin, channelAmount);

Servo servo_buoy;
Servo servo_wind;

float roll;
float pitch;
float heading;


int getOutputSignal(int max_in, int max_out, int min_in, int min_out, int value){
  int x = ((max_out-min_out)*value+(min_out*max_in-min_in*max_out))/(max_in-min_in);
  return x;
}

void initialize_outputs(){
  pinMode(PumpLeftPin, OUTPUT);
  pinMode(PumpRightPin, OUTPUT);
  pinMode(PumpUpPin, OUTPUT);
  pinMode(PumpDownPin, OUTPUT);
  pinMode(GrasperPin, OUTPUT);
  pinMode(WindingPin, OUTPUT);
  ServoPos = 90;
  servo_buoy.attach(ServoPin);
  servo_wind.attach(WindingPin);
}


void initialize_SD(){
  if (!SD.begin(chipSelect)) {
    #ifdef DEBUG
    Serial.println("Card failed, or not present");
    #endif 
    // don't do anything more:
    while (1);
  }
  #ifdef DEBUG
  Serial.println("card initialized.");
  #endif 
  snprintf(fileName, sizeof(fileName), "%05d.txt", fileCounter);
  
  #ifdef DEBUG
    Serial.println(fileCounter);
    Serial.println(fileName);
  #endif 
}

void initialize_IMU(){
  myIMU.begin();
  calibrateIMU(250, 250);
  lastTime = micros();
  filter.begin(25);
}

void read_RC(){
  chPitch_value = ppm.rawChannelValue(ch_pitch);//chPitch_ forward
  chBuoyancy_value = ppm.rawChannelValue(ch_buoyancy);//chBuoyancy_ turn
  chThrottle_value = ppm.rawChannelValue(ch_throttle);//chThrottle_ pitch
  chTurn_value = ppm.rawChannelValue(ch_turn);//chTurn_ Boyancy 
  chWinding_value = ppm.rawChannelValue(ch_winding);//chTurn_ Boyancy
  chGrasping_value = ppm.rawChannelValue(ch_grasping);//chTurn_ Boyancy
}

void save_SD(){
  String dataString = "";
  float data_SD[DATA_ON_SD] = {forwardRate, Windingval, chPitch_value, chBuoyancy_value, chThrottle_value, chTurn_value, chTurn_value, chGrasping_value, PumpLeftRate, PumpRightRate, PumpUpRate, PumpDownRate, ServoPos, GrasperPin, degreesX, degreesY, degreesZ}; //add IMU
  for (int counter = 0; counter < DATA_ON_SD; counter++) {
    dataString += String(data_SD[counter]);
    if (counter < DATA_ON_SD-1) {
      dataString += ",";
    }
  }
  
  File dataFile = SD.open(fileName, FILE_WRITE);
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    #ifdef DEBUG
      Serial.println(dataString);
    #endif
  }
  // if the file isn't open, pop up an error:
  #ifdef DEBUG
    else {
      Serial.println("error opening file");
    }
  #endif
}

void get_IMU(){
  myIMU.readAcceleration(accelX, accelY, accelZ);
  myIMU.readGyroscope(gyroX, gyroY, gyroZ);
  myIMU.readMagneticField(magX, magY, magZ);
  unsigned long currentTime = micros();
  lastInterval = currentTime - lastTime;  // expecting this to be ~104Hz +- 4%
  lastTime = currentTime;

  // Apply mag offset compensation (base values in uTesla)
  float x = magX - mag_offsets[0];
  float y = magY - mag_offsets[1];
  float z = magZ - mag_offsets[2];

  // Apply mag soft iron error compensation
  float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
  float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
  float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

  // Apply gyro zero-rate error compensation
  float gx = gyroX - gyro_zero_offsets[0];
  float gy = gyroY - gyro_zero_offsets[1];
  float gz = gyroZ - gyro_zero_offsets[2];

  // Update the filter
  filter.update(gx * dpsToRad, gy * dpsToRad, gz * dpsToRad,
                accelX, accelY, accelZ,
                mx, -my, -mz);

  // Print the orientation filter output
  if (millis() - a >= 250) {
    a += 250;
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    if (roll <= 0){
      roll=roll+360.0;}
  }
}

void get_pumps_out(){
  PumpLeftRate = TurnRate;
  PumpRightRate = -TurnRate;

  PumpUpRate = PitchRate;
  PumpDownRate = -PitchRate;

  if(forwardRate>=0){
    PumpLeftRate = PumpLeftRate+forwardRate;
    PumpRightRate = PumpRightRate+forwardRate;
  }
  else{
    PumpUpRate = PumpUpRate-forwardRate;
    PumpDownRate = PumpDownRate-forwardRate;
  }

  if(PumpLeftRate>255){PumpLeftRate = 255;}
  if(PumpLeftRate<0){PumpLeftRate = 0;}
  if(PumpRightRate>255){PumpRightRate=255;}
  if(PumpRightRate<0){PumpRightRate=0;}

  if(PumpUpRate>255){PumpUpRate = 255;}
  if(PumpUpRate<0){PumpUpRate = 0;}
  if(PumpDownRate>255){PumpDownRate=255;}
  if(PumpDownRate<0){PumpDownRate=0;}
}
void write_outputs(){
  servo_buoy.write(ServoPos);
  analogWrite(PumpLeftPin,PumpLeftRate);
  analogWrite(PumpRightPin,PumpRightRate);
  analogWrite(PumpUpPin,PumpUpRate);
  analogWrite(PumpDownPin,PumpDownRate);
  servo_wind.write(Windingval);

  if (chGrasping_value > 1600){
    digitalWrite(GrasperPin,HIGH);
    delay(100);
  }
  else{
    digitalWrite(GrasperPin,LOW);
    delay(100);
  }
  digitalWrite(PumpDownPin,PumpDownRate);
}

void RC_2_control(){
  int ServoRate; 
  if ((chBuoyancy_value) < 1400 || (chBuoyancy_value) > 1600){
    ServoRate = getOutputSignal(chBuoyancy_max, 50, chBuoyancy_min, -50, chBuoyancy_value);
  }
  //int ServoRate1 = getOutputSignal(chBuoyancy_max, 50, chBuoyancy_min, -50, chGrasping_value);

  ServoPos = ServoPos+ServoRate;
  if (ServoPos>179){ServoPos=185;}
  else if(ServoPos<3){ServoPos=-5;}
  
  TurnRate = getOutputSignal(chTurn_max,255,chTurn_min,-255,chTurn_value);
  PitchRate = getOutputSignal(chPitch_max,255,chPitch_min,-255,chPitch_value);
  forwardRate = getOutputSignal(chThrottle_max,255,chThrottle_min,0,chThrottle_value);
  Windingval = map(chWinding_value, 1000, 2000, 0, 180);
}

void read_IMU(){
  
}

void print_to_serial(){
  Serial.println(forwardRate);
  Serial.println(chPitch_value);
  Serial.println(chBuoyancy_value);
  Serial.println(chThrottle_value);
  Serial.println(chTurn_value);
  Serial.println(Windingval);
  Serial.println(chGrasping_value);
}
  