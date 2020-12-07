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

typedef struct  {
  uint8_t x;
  uint8_t y;
  uint8_t z;
} accelData;

void LSM303init();
void LSM303getAccel(uint16_t * accelData);

#endif