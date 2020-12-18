#ifndef BLINKY_H
#define BLINKY_H

#define DEBUG 1

#define DBGPRINT(...) \
            do { if (DEBUG) printf(__VA_ARGS__); } while (0)

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf.h"
#include "bsp.h"
#include "nrf_drv_twi.h"
#include "app_util_platform.h"
#include "i2c.h"
#include "LSM303AGR.h"

#endif