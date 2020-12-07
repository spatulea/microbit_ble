#include "blinky.h"

static uint8_t writeRegData[2] = {0, 0};
 
// static uint8_t readRegData[2];

void LSM303init() {

  // Set 10Hz rate and enable all axes
  writeRegData[0] = ACCEL_REG_CTRL1;
  // writeRegData[1] = 0b00000111;
  writeRegData[1] = 0x57;
  i2cWrite(ACCEL_I2C_ADDRESS,writeRegData,2);
  // TODO add a write/verify option and check for failures

  // Set
}

void LSM303getAccel(accelData_t * accelData) {
  uint8_t accelRawData[6] = {0,0,0,0,0,0};

  i2cRead(ACCEL_I2C_ADDRESS,(ACCEL_REG_OUT_X_L | ACCEL_SEQ_READ_BIT),accelRawData,6);

  // Convert the two left-justified uint_8 to a single int16_t
  accelData->x = (*(int16_t*) &accelRawData[0]) >> 6;
  accelData->y = (*(int16_t*) &accelRawData[2]) >> 6;
  accelData->z = (*(int16_t*) &accelRawData[4]) >> 6;

}