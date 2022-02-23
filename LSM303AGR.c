#include "microbit_ble.h"

static uint8_t writeRegData[2] = {0, 0};
// Calibration values for accelerometer
static calValues_t LSM303calData;
 
// static uint8_t readRegData[2];
void LSM303init() {

  // Set 10Hz rate and enable all axes
  writeRegData[0] = ACCEL_REG_CTRL1;
  // 10 Hz
  // writeRegData[1] = 0b00100111;
  // 100Hz
  // writeRegData[1] = 0b01010111;
  // 200Hz
  writeRegData[1] = 0b01100111;


  // writeRegData[1] = 0x57;
  i2cWrite(ACCEL_I2C_ADDRESS,writeRegData,2);
  // TODO add a write/verify option and check for failures

  // Set
}

void LSM303getAccel(accelData_t * accelData) {
  uint8_t accelRawData[6] = {0,0,0,0,0,0};
  int16_t accelSumData[3] = {0,0,0};

  for (uint8_t i = 0; i < ACCEL_NUM_AVERAGES; i++) {
    // Get new accel reading when available
    while(!LSM303dataReady());
    i2cRead(ACCEL_I2C_ADDRESS,(ACCEL_REG_OUT_X_L | ACCEL_SEQ_READ_BIT),accelRawData,6);
    // Convert reading to usable 16-bit format and sum
    accelSumData[0] += (*(int16_t*) &accelRawData[0]) >> 6;
    accelSumData[1] += (*(int16_t*) &accelRawData[2]) >> 6;
    accelSumData[2] += (*(int16_t*) &accelRawData[4]) >> 6;
  }
  // DBGPRINT("split: %#x,%#x\n\r",accelRawData[1],accelRawData[0]);
  // DBGPRINT("combined: %#x\n\r",((*(int16_t*) &accelRawData[0]) >> 6));
  // Convert the two left-justified uint_8 to a single int16_t
  // accelData->x = LSM303bitsToMg((*(int16_t*) &accelRawData[0]) >> 6);
  // accelData->y = LSM303bitsToMg((*(int16_t*) &accelRawData[2]) >> 6);
  // accelData->z = LSM303bitsToMg((*(int16_t*) &accelRawData[4]) >> 6);
  accelData->x = LSM303bitsToMg(accelSumData[0] >> ACCEL_AVG_SHIFT);
  accelData->y = LSM303bitsToMg(accelSumData[1] >> ACCEL_AVG_SHIFT);
  accelData->z = LSM303bitsToMg(accelSumData[2] >> ACCEL_AVG_SHIFT);


  // Apply calibration
  if (LSM303calData.calibrated) {
    accelData->x = (((accelData->x - LSM303calData.xMin) * ACCEL_RANGE_REF) /
                    LSM303calData.xRange) + ACCEL_MIN_REF;
    accelData->y = (((accelData->y - LSM303calData.yMin) * ACCEL_RANGE_REF) /
                  LSM303calData.yRange) + ACCEL_MIN_REF;
    accelData->z = (((accelData->z - LSM303calData.zMin) * ACCEL_RANGE_REF) /
                    LSM303calData.zRange) + ACCEL_MIN_REF;
  }
}

// Check if new data is ready
// TODO: switch to using pin-based interrupt
bool LSM303dataReady() {
  uint8_t regData = 0;

  i2cRead(ACCEL_I2C_ADDRESS, ACCEL_REG_STATUS, &regData, 1);

  regData &= 0b00001000;

  return(regData > 0 ? true : false);
}

float LSM303bitsToMg(int16_t bits) {
  return (((float)bits) * 3.9f / 1000.0f) * 9.8f;
}

void mgToDeg(accelData_t * accelData, accelData_t * inclinationData) {
  // Convert acceleration to inclination in degrees
  // -------------------------------

  // // Convert to double floating point
  // double accelX = double(acceleration[0])/1000.0;
  // double accelY = double(acceleration[1])/1000.0;
  // double accelZ = double(acceleration[2])/1000.0;

  // Get theta, psi and phi
  // Probably only need theta and psi (phi? second one)
  inclinationData->x = atan(accelData->x /
                            (sqrt((accelData->y) * (accelData->y) + (accelData->z) * (accelData->z)))) *
                       180 / M_PI;
  inclinationData->y = atan(accelData->y /
                            (sqrt((accelData->x) * (accelData->x) + (accelData->z) * (accelData->z)))) *
                       180 / M_PI;
  inclinationData->z = atan((sqrt((accelData->x) * (accelData->x) + (accelData->y) * (accelData->y))) /
                            accelData->z) *
                       180 / M_PI;
}
// -------------------------------


// Load default/ideal cal values
void LSM303defaultCal() {
  LSM303calData.calibrated = false;

  LSM303calData.xMin = -9.8f;
  LSM303calData.yMin = -9.8f;
  LSM303calData.zMin = -9.8f;

  LSM303calData.xMax = 9.8f;
  LSM303calData.yMax = 9.8f;
  LSM303calData.zMax = 9.8f;

  LSM303calData.xRange = 19.6f;
  LSM303calData.yRange = 19.6f;
  LSM303calData.zRange = 19.6f;
}

void LSM303calibrate() {

  accelData_t accelData;

  // Zero out any previous calibration values
  LSM303calData.xMin = 0.0f;
  LSM303calData.yMin = 0.0f;
  LSM303calData.zMin = 0.0f;

  LSM303calData.xMax = 0.0f;
  LSM303calData.yMax = 0.0f;
  LSM303calData.zMax = 0.0f;

  LSM303calData.xRange = 0.0f;
  LSM303calData.yRange = 0.0f;
  LSM303calData.zRange = 0.0f;

  DBGPRINT("Starting calibration...\n\r");
  DBGPRINT("Move all axes toward/away from gravity\n\r");

    //TODO: replace with a better check for complete calibration
    //  Maybe look for small delta in newest min/max readings
  for (int i = 0; i < 300; i++) {
    LSM303getAccel(&accelData);
    
    // Update minimums
    if (accelData.x < LSM303calData.xMin) LSM303calData.xMin = accelData.x;
    if (accelData.y < LSM303calData.yMin) LSM303calData.yMin = accelData.y;
    if (accelData.z < LSM303calData.zMin) LSM303calData.zMin = accelData.z;

    // Update maximums
    if (accelData.x > LSM303calData.xMax) LSM303calData.xMax = accelData.x;
    if (accelData.y > LSM303calData.yMax) LSM303calData.yMax = accelData.y;
    if (accelData.z > LSM303calData.zMax) LSM303calData.zMax = accelData.z;
    DBGPRINT("%6.2f,%6.2f | %6.2f,%6.2f | %6.2f,%6.2f\n\r",
      LSM303calData.xMin, LSM303calData.xMax,
      LSM303calData.yMin, LSM303calData.yMax,
      LSM303calData.zMin, LSM303calData.zMax);
  }
  
  // Compute the range
  LSM303calData.xRange = LSM303calData.xMax - LSM303calData.xMin;
  LSM303calData.yRange = LSM303calData.yMax - LSM303calData.yMin;
  LSM303calData.zRange = LSM303calData.zMax - LSM303calData.zMin;

  LSM303calData.calibrated = true;

}