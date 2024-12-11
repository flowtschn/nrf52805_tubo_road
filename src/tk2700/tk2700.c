/*
 *  tk2700.c
 *
 *  Created on: Dec 18, 2023
 */

/******************************************************************************/

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include "tk2700.h"

LOG_MODULE_REGISTER(tk2700, 3);

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

#define TK2700_ADDRESS 0x6D

#define MEASURE_REG    0x30
#define READ_REG       0x06

#define MEASURE_CMD    0x0A

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

static const struct i2c_dt_spec tk2700_bus = {
    .bus = DEVICE_DT_GET(DT_NODELABEL(i2c0_inst)),
    .addr = TK2700_ADDRESS
};

/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/



/******************************************************************************/

/*!
 * @brief  Start measurement and read the results
 */
int tk2700_read_results(int16_t *pressure, int16_t *temperature) {
    uint8_t buf[5];
    uint8_t cmd = MEASURE_CMD;

    *pressure = 0;
    *temperature = 0;

    /* Start measurement command */
    if (i2c_burst_write_dt(&tk2700_bus, MEASURE_REG, &cmd, 1) != 0) {
        LOG_ERR("Start measurement failed");
        return -1;
    }

    /* It takes about 3.8ms to complete a measurement. Host has to wait 4.5ms before sending the read data */
    k_msleep(10);

    /* Get results */
    if (i2c_burst_read_dt(&tk2700_bus, READ_REG, buf, sizeof(buf)) != 0) {
        LOG_ERR("Read results failed");
        return -2;
    }

    /* Convert kPa */
    *pressure = (int16_t) ((buf[0] << 8) | (buf[1]));

    /* Convert 2 bytes to 0.1C temperature. Byte 1 is integer number, byte 2 is decimal (resolution is 1/256 C) */
    *temperature = (int16_t) ((int8_t) buf[3] * 10);
    if (*temperature >= 0) {
        *temperature += (buf[4] * 10 / 256);
    }
    else {
        
        *temperature -= (buf[4] * 10 / 256);
    }
    return 0;
}

/*!
 * @brief  Initialize TK2700
 */
int tk2700_init(void) {
    /* Check I2C bus is ready */
    if (!device_is_ready(tk2700_bus.bus)) {
        LOG_ERR("I2C0 bus device not ready");
        return -1;
    }

    return 0;
}
