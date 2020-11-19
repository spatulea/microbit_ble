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
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"


// const uint8_t leds_list[LEDS_NUMBER] = LEDS_LIST;inte

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    // Configure LED-pins as outputs.
    // LEDS_CONFIGURE(LEDS_MASK);
    bool toggle = false;

    // toggle = true;

    // for (int i=1;1<5;i++) {
    //     toggle = !toggle;
    // }

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

    nrf_delay_ms(1000);

    nrf_delay_ms(100);

    nrf_delay_us(10);

    int check = 1;

    // Toggle LEDs.
    while (check<128)
    {
        // for (int i = 0; i < LEDS_NUMBER; i++)
        // {
        //     LEDS_INVERT(1 << leds_list[i]);
        //     nrf_delay_ms(500);
        // }

        // Blink row 1
        if (toggle) nrf_gpio_pin_set(13);
        else nrf_gpio_pin_clear(13);
        toggle = !toggle;
        nrf_delay_ms(100);
        check++;
        
    }
}


/** @} */
