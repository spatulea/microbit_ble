#include "blinky.h"

static uint8_t writeRegData[2] = {0, 0};
 
// static uint8_t readRegData[2];

void LSM303init() {

  // Set 10Hz rate and enable all axes
  writeRegData[0] = ACCEL_REG_CTRL1;
  writeRegData[1] = 0b00000111;
  i2cWrite(ACCEL_I2C_ADDRESS,writeRegData,2);
  // TODO add a write/verify option and check for failures

  // Set
}

void LSM303getAccel(uint16_t * accelData) {
  // struct {
  //   uint8_t xL;
  //   uint8_t xH;
  //   uint8_t yL;
  //   uint8_t yH;
  //   uint8_t zL;
  //   uint8_t zH;
  // } accelRawData;

  uint8_t accelRawData[6] = {0,0,0,0,0,0};
  // (uint8_t *)&

  i2cRead(ACCEL_I2C_ADDRESS,ACCEL_REG_OUT_X_L,accelRawData,1);
  nrf_delay_ms(1);  

}