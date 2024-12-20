/*
 *  xgzp6812d.c
 *
 *  Created on: Dec 18, 2024
 */

/******************************************************************************/

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include "xgzp6812d.h"

LOG_MODULE_REGISTER(xgzp6812d, 3);

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

#define XGZP6812D_ADDRESS 0x78
#define READ_REG  XGZP6812D_ADDRESS << 1 | 0x01
#define WRITE_REG XGZP6812D_ADDRESS << 1

#define DELAY_TIME 20
#define PMIN 0          // The minimum range pressure value for example 30Kpa
#define PMAX 700.0      // The full scale pressure value, for example 700Kpa
#define DMIN 1677722.0  // AD value corresponding to The minimum range pressure,
                        // for example 10%AD=2^24*0.1
#define DMAX 15099494.0 // AD Value Corresponding to The full scale pressure value,
                        // for example 90%AD=2^24*0.9


float pressure = 0.0; //KPa
float temperature = 0.0; //℃

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

static const struct i2c_dt_spec xgzp6812d_bus = {
    .bus = DEVICE_DT_GET(DT_NODELABEL(i2c0_inst)),
    .addr = XGZP6812D_ADDRESS
};

/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/


//Read the status of the sensor and judge whether IIC is busy
uint8_t xgzp6812d_isbusy(void)
{
    uint8_t status = 0;

    if (i2c_burst_read_dt(&xgzp6812d_bus, READ_REG, &status, 1) != 0) {
        LOG_ERR("Getting busy status failed!");
        return -1;
    }

    status = (status >> 5) & 0x01;
    return status;
}


int xgzp6812d_get_cal(float *pressure, float *temperature) //The function of reading pressure and temperature from the sensor
{ 
    uint8_t buffer[6] = {0}; //Temp variables used to restoring bytes from the sensor
    uint32_t Dtest = 0;
    uint16_t temp_raw = 0;
    buffer[0] = 0xAC; //Send 0xAC command and read the returned six-byte data

    if (i2c_burst_write_dt(&xgzp6812d_bus, WRITE_REG, buffer, 1) != 0) {
        LOG_ERR("get cal write failed!");
        return -1;
    }

    k_msleep(DELAY_TIME);
    while (1)
    {
        if (xgzp6812d_isbusy())
        {k_msleep(DELAY_TIME); }
        else
        break;
    }

    if (i2c_burst_read_dt(&xgzp6812d_bus, READ_REG, buffer, 6) != 0) {
        LOG_ERR("get cal read failed!");
        return -1;
    }

    //Computing the calibrated pressure and temperature values 
    Dtest = (uint32_t)((((uint32_t)buffer[1]) << 16) | (((uint16_t)buffer[2]) << 8) | ((uint8_t)buffer[3]));
    temp_raw = ((uint16_t)buffer[4] << 8) | (buffer[5] << 0);
    //The calibrated pressure value is converted into actual values
    if (Dtest != 0)
    { 
        *pressure = (float) ((PMAX-PMIN)/(DMAX-DMIN)*(Dtest-DMIN)+PMIN); //KPa
    }
    else
    {
        *pressure = 0.0; //pressure value, its unit is KPa 
        return 1;
    }

    float temp = (float) temp_raw / 65536; //The calibrated temperature value is converted into actual values
    *temperature = (((float)temp * 19000) - 4000) / 100; // its unit is ℃

    return 0;

}


/*!
 * @brief  Initialize XGZP6812D
 */
int xgzp6812d_init(void) {
    /* Check I2C bus is ready */
    if (!device_is_ready(xgzp6812d_bus.bus)) {
        LOG_ERR("I2C0 bus device not ready");
        return -1;
    }

    return 0;
}
