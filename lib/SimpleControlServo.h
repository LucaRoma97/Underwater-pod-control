#include "Arduino_BMI270_BMM150.h"
#define DEBUG
#include <PPMReader.h>
#define DATA_ON_SD 26

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
