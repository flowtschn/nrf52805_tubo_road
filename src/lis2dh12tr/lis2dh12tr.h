/*
 *  lis2dh12tr.h
 *
 *  Created on: Dec 18, 2023
 */

#ifndef _LIS2DH12TR_H_
#define _LIS2DH12TR_H_

/******************************************************************************/

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

struct lis2dh12tr_data {
    float x;
    float y;
    float z;
};

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/



/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/

/*!
 * @brief  Waiting for interrupt from sensor
 * @param  timeout
 * @retval 0 if success or error
 */
int lis2dh12tr_wait_interrupt(k_timeout_t timeout);

/*!
 * @brief  Configure sensor in interrupt mode
 * @param  timeout
 * @retval 0 if success or error
 */
int start_interrupt_mode(void);

/*!
 * @brief  Power down.
 */
int lis2dh12tr_power_down(void);

/*!
 * @brief  Initialize LIS2DH12TR
 * @param  None
 * @retval 0 if success or error
 */
int lis2dh12tr_init(void);

/******************************************************************************/

#endif /* _LIS2DH12TR_H_ */