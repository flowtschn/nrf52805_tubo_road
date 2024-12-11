 /*
 *  lis2dh12tr.c
 *
 *  Created on: Dec 18, 2023
 */

/******************************************************************************/

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include "lis2dh12_reg.h"
#include "lis2dh12tr.h"

LOG_MODULE_REGISTER(lis2dh12tr, 3);

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

#define LIS2DH12TR_ADDRESS 0x18
#define BOOT_TIME            5 //ms
/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

/* I2C bus */
static const struct i2c_dt_spec lis2dh12tr_bus = {
    .bus = DEVICE_DT_GET(DT_NODELABEL(i2c0_inst)),
    .addr = LIS2DH12TR_ADDRESS
};

/* Interrupt to wake up */
static const struct gpio_dt_spec accel_int1 = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(accelint1), gpios, 0);
static struct gpio_callback accel_int_cb;

/* Interrupt queue */
K_MSGQ_DEFINE(ext_queue, sizeof(uint8_t), 1, 4);

static stmdev_ctx_t dev_ctx;

/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/



/******************************************************************************/

/*!
 * @brief  Initialize external interrupt from INT1 pin
 */
static void ext_interrupt_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    uint8_t pin = pins;
    k_msgq_put(&ext_queue, &pin, K_NO_WAIT);
}


/*!
 * @brief  Write generic device register
 */
static int platform_write(void *handle, uint8_t reg, const uint8_t *buf, uint16_t len) {
    reg |= 0x80;
    return i2c_burst_write_dt(&lis2dh12tr_bus, reg, buf, len);
}

/*!
 * @brief  Read generic device register
 */
static int platform_read(void *handle, uint8_t reg, uint8_t *buf, uint16_t len) {
    reg |= 0x80;
    return i2c_burst_read_dt(&lis2dh12tr_bus, reg, buf, len);
}

/*!
 * @brief  Delay generic device register
 */
static void platform_delay(uint32_t ms) {
    k_msleep(ms);
}

/*!
 * @brief  Configure sensor in interrupt mode
 */
int start_interrupt_mode(void) {
    int ret;
    lis2dh12_int1_cfg_t int1_cfg;
    lis2dh12_ctrl_reg3_t ctrl_reg3;

    /* Disable pull-up on SDO/SA0 pin */
    lis2dh12_pin_sdo_sa0_mode_set(&dev_ctx, LIS2DH12_PULL_UP_DISCONNECT);

    /* Set data rate to 50Hz */
    lis2dh12_data_rate_set(&dev_ctx, LIS2DH12_ODR_400Hz);

    /* Set full scale to 8g */
    lis2dh12_full_scale_set(&dev_ctx, LIS2DH12_8g);

    /* Disable temperature measurement */
    lis2dh12_temperature_meas_set(&dev_ctx, LIS2DH12_TEMP_DISABLE);

    /* Set operating mode to low power */
    lis2dh12_operating_mode_set(&dev_ctx, LIS2DH12_LP_8bit);

    /* Set interrupt threshold to 496mg (~500) (1LSB = 62mg @8g FS) Normal=8 17*/
    ret = lis2dh12_int1_gen_threshold_set(&dev_ctx, 19);
    if (ret != 0) LOG_ERR("lis2dh12_int1_gen_threshold_set failed, error = %d", ret);

    /* Set interrupt duration to 10ms (1 LSB = 1/ODR) */
    ret = lis2dh12_int1_gen_duration_set(&dev_ctx, 1);
    if (ret != 0) LOG_ERR("lis2dh12_int1_gen_duration_set failed, error = %d", ret);

    /* Configure motion detection on all axis, high-pass filter enabled */
    memset((uint8_t *)&int1_cfg, 0, sizeof(int1_cfg));
    //int1_cfg.xhie = PROPERTY_ENABLE; 
    int1_cfg.yhie = PROPERTY_ENABLE;
    //int1_cfg.zhie = PROPERTY_ENABLE;
    ret = lis2dh12_int1_gen_conf_set(&dev_ctx, &int1_cfg);
    if (ret != 0) LOG_ERR("lis2dh12_int1_gen_conf_set failed, error = %d", ret);

    /* Enable interrupt generation on INT1 pin */
    memset((uint8_t *)&ctrl_reg3, 0, sizeof(ctrl_reg3));
    ctrl_reg3.i1_ia1 = PROPERTY_ENABLE; 
    ret = lis2dh12_pin_int1_config_set(&dev_ctx, &ctrl_reg3);
    if (ret != 0) LOG_ERR("lis2dh12_pin_int1_config_set failed, error = %d", ret);

    return ret;
}

/******************************************************************************/

/*!
 * @brief  Waiting for interrupt from sensor
 */
int lis2dh12tr_wait_interrupt(k_timeout_t timeout) {
    uint8_t pin;
    return k_msgq_get(&ext_queue, &pin, timeout);
}

/*!
 * @brief  Power down.
 */
int lis2dh12tr_power_down(void)
{
    int ret = lis2dh12_data_rate_set(&dev_ctx, LIS2DH12_ODR_10Hz);
    if (ret != 0) {
        LOG_ERR("lis2dh12_data_rate_set failed, error = %d", ret);
        return ret;
    }
    return 0;
}

/*!
 * @brief  Initialize LIS2DH12TR
 */
int lis2dh12tr_init(void) {
    int err;
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg  = platform_read;
    dev_ctx.mdelay    = platform_delay;
    dev_ctx.handle    = NULL;

    /* Check I2C bus is ready */
    if (!device_is_ready(lis2dh12tr_bus.bus)) {
        LOG_ERR("I2C0 bus device not ready");
        return -1;
    }

    platform_delay(BOOT_TIME);

    /* Check sensor is available */
    uint8_t whoamI;
    lis2dh12_device_id_get(&dev_ctx, &whoamI);
    if (whoamI != LIS2DH12_ID) {
        LOG_ERR("Couldn't get LIS2DH12 ID %02X != %02X", LIS2DH12_ID, whoamI);
        return -2;
    }

    /* Check interrupt is ready */
    if (!gpio_is_ready_dt(&accel_int1)) {
        LOG_ERR("Interrupt device not ready");
        return -3;
    }

    err = gpio_pin_configure_dt(&accel_int1, GPIO_INPUT);
    if (err) {
        LOG_ERR("Failed to configure intput pin (err %d)", err);
        return -4;
    }

    err = gpio_pin_interrupt_configure_dt(&accel_int1, GPIO_INT_EDGE_BOTH);
    if (err) {
        LOG_ERR("Failed to configure intput interrupt (err %d)", err);
        return -5;
    }

    /* Attach interrupt functions */
    gpio_init_callback(&accel_int_cb, ext_interrupt_callback, BIT(accel_int1.pin));
    gpio_add_callback(accel_int1.port, &accel_int_cb);
    
    uint8_t pin = 2;
    k_msgq_put(&ext_queue, &pin, K_NO_WAIT);    /* Push the first time to adv after power up */

    /* Start sensor in the interrupt mode */
    start_interrupt_mode();
    return 0;
}