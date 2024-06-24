/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/logging/log.h>
#include <app/drivers/cad_qspi.h>
#include <zephyr/drivers/pwm.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000
#define CCDRIVER	DT_ALIAS(my_ccdrive)
#define CNVDRIVER	DT_ALIAS(my_cnvdrive)
#define QSPIINSTANCE    DT_ALIAS(my_qspi)
#define CNV_QSPI_IRQ_ID 28 /* ISR Number*/
#define CNV_QSPI_IRQ_PRIO 35 /* ISR Priority*/
#define CNV_QSPI_IRQ_FLAGS  0

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

static const struct pwm_dt_spec ccDriver = PWM_DT_SPEC_GET(CCDRIVER); /*Get a device structure for the pwm device defined in the device tree overlay*/ 
static const struct pwm_dt_spec cnvDriver = PWM_DT_SPEC_GET(CNVDRIVER);
static const struct device* qspi_device = DEVICE_DT_GET(QSPIINSTANCE);
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

/* Function Forward Declaration */
static int generateSignals(void);

uint16_t rx_data = 1;
bool data_valid;

int main(void)
{
	int ret;
	bool led_state = true;
    data_valid = false;
    printk("Generating PWM Signals\n");
    ret = generateSignals();

	while (1) {
        //data_valid = qspi_read(qspi_device, &rx_data, 2)
        if(true){
            printk("Returned data: %d\n", rx_data);
            data_valid = false;
        }
        k_msleep(SLEEP_TIME_MS);
	}
	return 0;
}

static int generateSignals(){

    int ret;
    // ClotChip Drive Signal 
    ret = pwm_set_cycles(ccDriver.dev, ccDriver.channel, 64, 32, ccDriver.flags);
        if (ret < 0){
            printk("Error %d: failed to set pulse width\n", ret);
            return 0;
        }
    
    // ADC CNV Signal
    ret = pwm_set_cycles(cnvDriver.dev, cnvDriver.channel,64, 21, cnvDriver.flags);
        if (ret < 0){
            printk("Error %d: failed to set pulse width\n", ret);
            return 0;
        }

    return ret;
}