#ifndef I2C_H
#define I2C_H

void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context);
void i2cInit(const nrf_drv_twi_t * i2cPointer);
void i2cWrite(uint8_t i2cAddress , uint8_t * writeData, uint8_t numBytes);
void i2cRead(uint8_t i2cAddress, uint8_t readRegAddress, uint8_t * readData, uint8_t numBytes);

#endif