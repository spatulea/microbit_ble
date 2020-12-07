#include "blinky.h"

static const nrf_drv_twi_t * i2cInstance;

// Globals needed for handler-based reads
// Not my favorite implementation, but also not sure why
// Nordic chose to do reads this way
static bool g_i2cDoRead = false;
static bool g_i2cDoWrite = false;

static uint8_t g_numReadBytes = 0;
static uint8_t * g_readDataPtr;

static uint8_t g_i2cAddress = 0;

void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{   


    // printf("Event handler called!\r\n");
    switch(p_event->type)
    {
        case NRF_DRV_TWI_TX_DONE:
          // If there was a read request...
          if (g_i2cDoRead) {
            nrf_drv_twi_rx(i2cInstance, g_i2cAddress, g_readDataPtr, g_numReadBytes, false);
          }

          // write is done, let the blocking end
          g_i2cDoWrite = false;
          break;

        case NRF_DRV_TWI_ERROR:
          printf("\r\n!****\r\nTWI Event Error\r\n");
            g_i2cDoRead = false;
          break;

        case NRF_DRV_TWI_RX_DONE:
          // Read is done, let the blocking end
          g_i2cDoRead = false;
          break;
        // case NRF_DRV_TWI_EVT_ADDRESS_NACK:
        //     printf("No address ACK on address: %#x!\r\n", device_address);
        //     break;
        // case NRF_DRV_TWI_EVT_DATA_NACK:
        //     printf("No data ACK on address: %#x!\r\n", device_address);
        //     break;
        default:
          break;
    }   
}

void i2cInit(const nrf_drv_twi_t * i2cPointer) {

    ret_code_t err_code;
    // Initialize accel/mag I2C ... nordic calls it "TWI"

    const nrf_drv_twi_config_t i2cConfig = {
       .scl                = SCL_PIN_NUMBER,
       .sda                = SDA_PIN_NUMBER,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH
    };

    i2cInstance = i2cPointer;

    err_code = nrf_drv_twi_init(i2cInstance, &i2cConfig, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(i2cInstance);

}

// Bit of a pointless wrapper function... but it's nice to keep it consistent with 
// the read function below
// writeData format is [ address, data, data, etc ]
void i2cWrite(uint8_t i2cAddress , uint8_t * writeData, uint8_t numBytes) {
    ret_code_t err_code;

    g_i2cDoWrite = true;

    err_code = nrf_drv_twi_tx(i2cInstance, i2cAddress, writeData, numBytes, false);

    // I like blocking functions for peripherals.. so we block until the write is done
    while(g_i2cDoWrite) {
      __WFE(); // TODO: check what __WFE really does
    }
    APP_ERROR_CHECK(err_code);
}

void i2cRead(uint8_t i2cAddress, uint8_t readRegAddress, uint8_t * readData, uint8_t numBytes) {
    ret_code_t err_code;

    // Let the twi handler know we want to read
    g_i2cDoRead = true;
    // Let the twi know where to store the read data
    g_i2cAddress = i2cAddress;
    g_readDataPtr = readData;
    g_numReadBytes = numBytes;

    // Write the register address to be read
    err_code = nrf_drv_twi_tx(i2cInstance, i2cAddress, &readRegAddress, 1, false);

    // I like blocking functions for reading some peripherals.. so we block until the read is done
    while(g_i2cDoRead){
      // __WFE(); // TODO: check what __WFE really does
    }
    APP_ERROR_CHECK(err_code);
}