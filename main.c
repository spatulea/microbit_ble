/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup blinky_example_main main.c
 * @{
 * @ingroup blinky_example
 * @brief Blinky Example Application main file.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf.h"
#include "bsp.h"
#include "nrf_drv_twi.h"
#include "app_util_platform.h"


// #include "nrf_drv_config.h"
// #include "boards.h"

#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 1                           /**< UART RX buffer size. */

uint8_t device_address = 0; // Address used to temporarily store the current address being checked
bool device_found = false; 


void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

// TWI (I2C) stuff
static const nrf_drv_twi_t i2c = NRF_DRV_TWI_INSTANCE(0);

/* Indicates if I2C reading operation has ended. */
static volatile bool m_xfer_done = true;
/* Indicates if I2C setting mode operation has ended. */
static volatile bool m_set_mode_done;

void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{   
    printf("Event handler called!\r\n");
    switch(p_event->type)
    {
        case NRF_DRV_TWI_TX_DONE:
            // If EVT_DONE (event done) is received a device is found and responding on that particular address
            printf("\r\n!****************************!\r\nDevice found at 7-bit address: %#x!\r\n!****************************!\r\n\r\n", device_address);
            device_found = true;
            break;
        case NRF_DRV_TWI_ERROR:
            printf("\r\n!****\r\nError at address %#x \r\n", device_address);
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

#define MMA8653_address 0x1E
#define MAG3110_address 0x19

/**
 * @brief Initialize I2C bus
 */

void i2cInit(void) {

    ret_code_t err_code;
    // Initialize accel/mag I2C ... nordic calls it "TWI"

    const nrf_drv_twi_config_t i2cConfig = {
       .scl                = 0,//SCL_PIN_NUMBER,
       .sda                = 30,//SDA_PIN_NUMBER,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH
    };

    err_code = nrf_drv_twi_init(&i2c, &i2cConfig, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&i2c);

}

void i2cWriteAccel(uint8_t * writeData) {
    ret_code_t err_code;

    err_code = nrf_drv_twi_tx(&i2c, MMA8653_address, writeData, sizeof(writeData),true);
    APP_ERROR_CHECK(err_code);

}

void i2cReadAccel(uint8_t * readData, uint8_t numBytes) {
    ret_code_t err_code;
    err_code = nrf_drv_twi_rx(&i2c, MMA8653_address, readData, numBytes, true);
    APP_ERROR_CHECK(err_code);

}

/**
 * @brief Function for application main entry.
 */
int main(void)
{

    bool toggle = false;
    // uint32_t err_code;
    ret_code_t err_code;

     // Initialize the debug UART
    const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          APP_UART_FLOW_CONTROL_ENABLED,
          false,
          UART_BAUDRATE_BAUDRATE_Baud115200
      };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOW,
                         err_code);

    APP_ERROR_CHECK(err_code);


    // Set all column GPIOs for the LEDs (I think) to high (off)
    for (int n=4;n<=12;n++) {
        nrf_gpio_cfg_output(n);
        nrf_gpio_pin_set(n);
    }

    // Set all row GPIOs for the LEDs to off as well
    for (int n=13;n<=15;n++) {

        nrf_gpio_cfg_output(n);
        nrf_gpio_pin_clear(n);
    }

    // Enable column 1
    nrf_gpio_pin_clear(4);

    nrf_delay_ms(10);

    // Do i2c stuff
    i2cInit();
    // uint8_t readData[1] = {0};
    uint8_t dummy_data = 0x55;
    // Itterate through all possible 7-bit TWI addresses
    for(uint8_t i = 0; i <= 0x7F; i++)
    {
        device_address = i;
        // Send dummy data. If a device is present on this particular address a TWI EVT_DONE event is 
        // received in the twi event handler and a message is printed to UART
        printf("Trying...%d\n\r",i);
        nrf_drv_twi_tx(&i2c, i, &dummy_data, 1, false);
        // Delay 10 ms to allow TWI transfer to complete and UART to print messages before starting new transfer
        nrf_delay_ms(10);
    }
    // if(device_found)
    // {
    //     // Blinke LED_1 rapidly if device is found
    //     while(true)
    //     {
    //         nrf_gpio_pin_toggle(LED_1);
    //         nrf_delay_ms(100);
    //     }
    // }
    // i2cReadAccel(readData,1)


    printf("\n\rStart: \n\r");

    while (true)
    {
        uint8_t cr;
        while(app_uart_get(&cr) != NRF_SUCCESS);
        while(app_uart_put(cr) != NRF_SUCCESS);

        if (cr == 'q' || cr == 'Q')
        {
            printf(" \n\rExit!\n\r");

            while (true)
            {
                if (toggle) nrf_gpio_pin_set(13);
                else nrf_gpio_pin_clear(13);
                toggle = !toggle;
                nrf_delay_ms(100);
            }
        }

    }
}


/** @} */
