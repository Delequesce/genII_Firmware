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
#include <app/drivers/ad4002.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/pwm.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS       1000
#define CCDRIVER	        DT_ALIAS(my_ccdrive)
#define AD4002_INSTANCE_1   DT_ALIAS(ad4002_ch1)
#define RX_BUFFER_LENGTH    2
#define V_SIG_PERIOD        64  /* In clock cycles */


/* Obtain Relevant Device Tree Structures */
static const struct pwm_dt_spec ccDriver = PWM_DT_SPEC_GET(CCDRIVER);
static const struct device* ad4002_device_1 = DEVICE_DT_GET(AD4002_INSTANCE_1);

/* Function Forward Declaration */
static int startDriveSignal(void);

bool data_valid;

int main(void)
{
	int ret;
    data_valid = false;

    /* Set up SPI buffer for AD4002 Reads */
    uint8_t rx_buffer[RX_BUFFER_LENGTH] = {0};
	struct spi_buf my_spi_buffer[1];
	my_spi_buffer[0].buf = rx_buffer;
	my_spi_buffer[0].len = RX_BUFFER_LENGTH;
	const struct spi_buf_set rx_buff = {my_spi_buffer, 1};

    printk("Generating PWM Signals\n");
    ret = startDriveSignal();

	while (1) {
        data_valid = ad4002_continuous_read(ad4002_device_1, &rx_buff);
        if(true){
            //printk("Returned data: %d\n", rx_buffer);
            data_valid = false;
        }
        k_msleep(SLEEP_TIME_MS);
	}
	return 0;
}

static int startDriveSignal(){

    int ret;
    // ClotChip Drive Signal 
    ret = pwm_set_cycles(ccDriver.dev, ccDriver.channel, V_SIG_PERIOD, V_SIG_PERIOD/2, ccDriver.flags);
        if (ret < 0){
            printk("Error %d: failed to set pulse width\n", ret);
            return 0;
        }
    return ret;
}