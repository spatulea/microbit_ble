#include "blinky.h"

static uint8_t writeRegData[2] = {0, 0};
 
// static uint8_t readRegData[2];

void LSM303init() {

  // Set 10Hz rate and enable all axes
  writeRegData[0] = ACCEL_REG_CTRL1;
  writeRegData[1] = 0b00100111;
  // writeRegData[1] = 0x57;
  i2cWrite(ACCEL_I2C_ADDRESS,writeRegData,2);
  // TODO add a write/verify option and check for failures

  // Set
}

void LSM303getAccel(accelData_t * accelData) {
  uint8_t accelRawData[6] = {0,0,0,0,0,0};

  i2cRead(ACCEL_I2C_ADDRESS,(ACCEL_REG_OUT_X_L | ACCEL_SEQ_READ_BIT),accelRawData,6);
  // printf("split: %#x,%#x\n\r",accelRawData[1],accelRawData[0]);
  // printf("combined: %#x\n\r",((*(int16_t*) &accelRawData[0]) >> 6));
  // Convert the two left-justified uint_8 to a single int16_t
  accelData->x = LSM303bitsToMg((*(int16_t*) &accelRawData[0]) >> 6);
  accelData->y = LSM303bitsToMg((*(int16_t*) &accelRawData[2]) >> 6);
  accelData->z = LSM303bitsToMg((*(int16_t*) &accelRawData[4]) >> 6);
}

bool LSM303dataReady() {
  uint8_t regData = 0;

  i2cRead(ACCEL_I2C_ADDRESS, ACCEL_REG_STATUS, &regData, 1);

  regData &= 0b00001000;

  return(regData > 0 ? true : false);
}

float LSM303bitsToMg(int16_t bits) {
  return (((float)bits /*/ 64.0f */) * 3.9f / 1000.0f) * 9.8f;
}

void mgToDeg(accelData_t * accelData, accelData_t * inclinationData) {
  // Convert acceleration to inclination in degrees
  // -------------------------------

  // // Convert to double floating point
  // // TODO confirm values are scaled by 1000. Confirmed this is wrong, but kinda OK for now. Should use the FS value to calculate correct gs
  // double accelX = double(acceleration[0])/1000.0;
  // double accelY = double(acceleration[1])/1000.0;
  // double accelZ = double(acceleration[2])/1000.0;

  // Get theta, psi and phi
  // Probably only need theta and psi (phi? second one)
  inclinationData->x = atan(accelData->x / (sqrt(exp2(accelData->y)+exp2(accelData->z))))*180/M_PI;
  inclinationData->y = atan(accelData->y / (sqrt(exp2(accelData->x)+exp2(accelData->z))))*180/M_PI;
  inclinationData->z = atan((sqrt(exp2(accelData->x)+exp2(accelData->y)))/accelData->z)*180/M_PI;
}
// -------------------------------