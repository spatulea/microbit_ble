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

#include "blinky.h"

#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 1                           /**< UART RX buffer size. */

static const nrf_drv_twi_t i2c = NRF_DRV_TWI_INSTANCE(0);

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

// #define MMA8653_address 0x1E

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
    i2cInit(&i2c);


    DBGPRINT("\n\rStart: \n\r");

    LSM303init();

    accelData_t accelData;
    accelData_t inclination;

    while (true)
    {
        uint8_t cr;
        while(app_uart_get(&cr) != NRF_SUCCESS);
        while(app_uart_put(cr) != NRF_SUCCESS);

        if (cr == 'g' || cr == 'G')
        {
            DBGPRINT("Reading accelerometer...\n\r");
            uint8_t i = 0;
            while(i < 100) {
                // Use software "interrupt" driven sampling set by the accelerometer
                // power mode / datarate setting for now
                // TODO: switch to interrupt-pin driven sampling and low-power mode
                if (LSM303dataReady()) {
                    LSM303getAccel(&accelData);
                    DBGPRINT("%6.2f, %6.2f, %6.2f\n\r",accelData.x,accelData.y,accelData.z);
                    //mgToDeg(&accelData,&inclination);
                    //DBGPRINT("%6.2f, %6.2f, %6.2f\n\r",inclination.x,inclination.y,inclination.z);

                    i++;
                }
            }
        }
        if (cr == 'h' || cr == 'H')
        {
            DBGPRINT("Reading accelerometer...\n\r");
            uint8_t i = 0;
            while(i < 100) {
                // Use software "interrupt" driven sampling set by the accelerometer
                // power mode / datarate setting for now
                // TODO: switch to interrupt-pin driven sampling and low-power mode
                if (LSM303dataReady()) {
                    LSM303getAccel(&accelData);
                    //DBGPRINT("%6.2f, %6.2f, %6.2f\n\r",accelData.x,accelData.y,accelData.z);
                    mgToDeg(&accelData,&inclination);
                    DBGPRINT("%6.2f, %6.2f, %6.2f\n\r",inclination.x,inclination.y,inclination.z);

                    i++;
                }
            }
        }
        if (cr == 'q' || cr == 'Q')
        {
            DBGPRINT(" \n\rExit!\n\r");

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
