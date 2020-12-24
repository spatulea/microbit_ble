#ifndef LSM303AGR_H
#define LSM303AGR_H

#define ACCEL_I2C_ADDRESS 0x19
#define MAG_I2C_ADDRESS   0x1E

#define ACCEL_REG_STATUS_AUX  0x07
#define ACCEL_REG_OUT_TEMP_L  0x0C
#define ACCEL_REG_OUT_TEMP_H  0x0D

#define ACCEL_REG_WHOAMI  0x0F
#define ACCEL_REG_TEMP_CFG_REG  0x1F

#define ACCEL_REG_CTRL1   0x20
#define ACCEL_REG_CTRL2   0x21
#define ACCEL_REG_CTRL3   0x22
#define ACCEL_REG_CTRL4   0x23
#define ACCEL_REG_CTRL5   0x24
#define ACCEL_REG_CTRL6   0x25

#define ACCEL_REG_STATUS  0x27

#define ACCEL_REG_OUT_X_L   0x28
#define ACCEL_REG_OUT_X_H   0x29

#define ACCEL_REG_OUT_Y_L   0x2A
#define ACCEL_REG_OUT_Y_H   0x2B

#define ACCEL_REG_OUT_Z_L   0x2C
#define ACCEL_REG_OUT_Z_H   0x2D

#define ACCEL_SEQ_READ_BIT  0b10000000

#define ACCEL_RANGE_REF 19.6f
#define ACCEL_MIN_REF -9.8f
#define ACCEL_MAX_REF 9.8f

typedef struct {
  float x;
  float y;
  float z;
} accelData_t;

typedef struct {
  bool calibrated;

  float xMin;
  float xMax;
  float xRange;
  
  float yMin;
  float yMax;
  float yRange;

  float zMin;
  float zMax;
  float zRange;
} calValues_t;


void LSM303init();
void LSM303getAccel(accelData_t * accelData);
bool LSM303dataReady();
float LSM303bitsToMg(int16_t bits);
void mgToDeg(accelData_t * accelData, accelData_t * inclinationData);
void LSM303defaultCal();
void LSM303calibrate();

#endif