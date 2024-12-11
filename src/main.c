/*
 *  main.c
 *
 *  Created on: Dec 17, 2023
 */

/******************************************************************************/

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/poweroff.h>
#include <hal/nrf_gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <helpers/nrfx_reset_reason.h>
#include "tk2700/tk2700.h"
#include "lis2dh12tr/lis2dh12tr.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

static int ADV_INTERVAL_S=30;    /* Advertising upto ADV_INTERVAL_S seconds if no motion detected */

#define MAX_PRESSURE            1000    /* Max is 10 bar so max value = 10 * 100 = 1000 kPA */
#define DEVICE_NAME_LENGTH      (sizeof(CONFIG_BT_DEVICE_NAME) + 5)    /* Prefix name + '-XXXX' */

#define SERVICE_UUID            0x181A      /* Custom service UUID */
#define IDX_TEMPL               4           /* Index of lo byte of temp in service data*/
#define IDX_TEMPH               5           /* Index of hi byte of temp in service data*/

#define BAT_ADC_CHANNEL_ID 0

//BT_GAP_ADV_SLOW_INT_MIN   0x0640 /* 1 s */
//BT_GAP_ADV_SLOW_INT_MAX   0x0780 /* 1.2 s */
#define ADV_PARAM BT_LE_ADV_PARAM(BT_LE_ADV_OPT_USE_IDENTITY, \
                  BT_GAP_ADV_SLOW_INT_MIN, \
                  BT_GAP_ADV_SLOW_INT_MAX, NULL)

/* Index in the service data */
#define PRES_INDEX              4
#define TEMP_INDEX              6

static int16_t pressure = 0, temperature = 0;
static double pressure_present = 0;
static double pressure_p2 = 0;
static volatile int sleep_counter = 0;
static volatile int ridemode_counter = 0; /* Counter for nowing system is in ridemode*/
static int16_t flagstart=0;
static int16_t stopble=0;


#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
	!DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified"
#endif

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
			     DT_SPEC_AND_COMMA)
};
/* adv_update_timer used to handle the sleep_counter without the need for k_msleep() */
static struct k_timer m_adv_update_timer_id;

/** adv update timer timeout callback. 
    This would be called when ever the (1000ms) inetrval passes. Making the sleep_ounter flag increases. 
*/
void adv_update_timer_timeout(struct k_timer *timer_id)
{
    sleep_counter++;
    LOG_INF("adv_update_timer_timeout sleep_counter %d", sleep_counter);
     

    ridemode_counter++;
    LOG_INF("ridemode_counter %d", ridemode_counter);

}

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

static uint8_t service_data[] = {
    BT_UUID_16_ENCODE(SERVICE_UUID),
    0x07,    /* Length */
    0x01,    /* Pressure type */
    0x00,    /* Value in 0.1 bar */
    0x02,    /* Temperature type */
    0x00,    /* Value in C */
};

static char device_name[DEVICE_NAME_LENGTH];
static struct bt_data adv_data[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
    BT_DATA(BT_DATA_NAME_COMPLETE, device_name, sizeof(device_name) - 1),
    BT_DATA(BT_DATA_SVC_DATA16, service_data, ARRAY_SIZE(service_data))
};


/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/



/******************************************************************************/

/*!
 * @brief  This callback is called after BT ready
 */
static void bt_ready(int err) {
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return;
    }
    LOG_INF("Bluetooth initialized");

    /* Update device name included mac address */
    bt_addr_le_t addr;
    size_t count = 1;
    bt_id_get(&addr, &count);
    sprintf(device_name, "%s-%02X%02X", CONFIG_BT_DEVICE_NAME, addr.a.val[4], addr.a.val[5]);
    LOG_INF("Device name %s", device_name);
}

void advertising_start()
{
     if (pressure_present>2)
     { 
        
    
    
    LOG_INF("Start advertising!!!");
    int err = bt_le_adv_start(ADV_PARAM, adv_data, ARRAY_SIZE(adv_data), NULL, 0);
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
        sys_reboot(SYS_REBOOT_WARM);
        return ;
    }
    flagstart=2;
    } 
    
} 




int main(void) {

    int err;
    LOG_INF("Power up! Firmware version %s", "1.0.0");
    k_msleep(100);    /* 1 seconds */

   

    /* Need to initialize lis2dh12tr to pull up INT1 pin */
    lis2dh12tr_init();
    if (lis2dh12tr_init() != 0) {
        LOG_INF("Failed to initialize lis2dh12tr. Restart system");
        k_msleep(100);
        sys_reboot(SYS_REBOOT_WARM);
    }

    /* Configure to generate PORT event (wakeup) on buttons press. */
    nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(DT_ALIAS(accelint), gpios), NRF_GPIO_PIN_PULLDOWN);
    nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(DT_ALIAS(accelint), gpios), NRF_GPIO_PIN_SENSE_HIGH);

    /* Initialize the Bluetooth Subsystem */
    err = bt_enable(bt_ready);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return 0;
    }

    /* the sensors */
    tk2700_init();

    /* Initialize the adv_update timer */
    k_timer_init(&m_adv_update_timer_id, adv_update_timer_timeout, NULL);


   
    /* Start the adv_update timer, periodic with 1000 ms as an interval */
    k_timer_start(&m_adv_update_timer_id, K_MSEC(1000), K_MSEC(1000));
   
    /* Loop */
    while (1) {
        if (ridemode_counter > 60) {
            stopble=1;
            bt_le_adv_stop();
            LOG_INF("Turn off ADV ");
        }
         if (ridemode_counter > 120) {
            advertising_start();
            stopble=0;
            LOG_INF("Turn on ADV ");
            ridemode_counter = 20;
        }   

        if (sleep_counter <= ADV_INTERVAL_S) 
        {
           
            
                if (tk2700_read_results(&pressure, &temperature) == 0) 
                {
                    

                        /*Meassuring the parameters*/
                        service_data[PRES_INDEX] = pressure / 10;                  /* kPa to 0.1bar */
                        service_data[TEMP_INDEX] = (int8_t) (temperature / 10);    /* C to 0.1C */
                        pressure_present=pressure/10;
                        LOG_INF("Temperature %d, pressure %d", temperature, pressure_present);

                         if (pressure_present>2)
                    { 

                        if ((pressure_p2 - pressure_present >= 2 || pressure_p2 > pressure_present) )
                        {
                            ADV_INTERVAL_S = 120;
                            sleep_counter = 0;
                            if (stopble==1) {
                                ridemode_counter=120;
                            }
                        }

                        if (flagstart<1)
                            {
                        advertising_start();    
                        
                            }

                        if (pressure > MAX_PRESSURE) {
                        pressure = MAX_PRESSURE;
                        }
                    
                
                        if(stopble==0){
                        /* Update adv */
                        err = bt_le_adv_update_data(adv_data, ARRAY_SIZE(adv_data), NULL, 0);
                        LOG_INF("Updated BLE Data");
                        if (err) {
                            LOG_ERR("Failed to update advertising data (err %d)", err);
                            }
                        }
                        pressure_p2=pressure_present;
                    }
                 if (pressure_present<2){
                        LOG_INF("Pressure ist to low- SHIPPING MODE");
                        sleep_counter=30;
                    } 
                } 
            
        
            
        } else {
            
            /* Stop advertising */
            bt_le_adv_stop();
            if (bt_le_adv_stop() == 0)
            {
               flagstart=0;     
            }
            /* Stop the adv_update timer */
            k_timer_stop(&m_adv_update_timer_id);
            /* Power down the acc sensor */
            lis2dh12tr_power_down();
            LOG_INF("Go to sleep...");
            k_msleep(100);    /* 100ms before sleeping */
            sys_poweroff();
        }

        /* If have interrupt, we reset timeout counter */
        if (lis2dh12tr_wait_interrupt(K_NO_WAIT) == 0) {
            sleep_counter = 0;
            LOG_INF("Sleep after %d seconds", ADV_INTERVAL_S);
        }

        //LOG_INF("END of While LOOP");
        k_msleep(1000);
    }
}