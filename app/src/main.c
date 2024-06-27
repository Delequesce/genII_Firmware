#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/sys/printk.h>
#include <app/drivers/ad4002.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/uart.h>
//#include <zephyr/usb/usb_device.h>
#include <string.h>

#define CCDRIVER	        DT_ALIAS(my_ccdrive)
#define AD4002_INSTANCE_1   DT_ALIAS(ad4002_ch1)
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

    if (startDriveSignal() < 0){
        return -1;
    }

    /* Begin Interrupt Driven ADC Read Operation */
    if (ad4002_continuous_read(ad4002_device_1) < 0){
        printk("Continuous Read Failure\n");
        return -1; 
    }
	while (1) {
        printk("Running\n");
        data_valid = false;
        k_msleep(1000);
	}
	return 0;
}

static int startDriveSignal(){

    int ret;
    // ClotChip Drive Signal 
    ret = pwm_set_cycles(ccDriver.dev, ccDriver.channel, V_SIG_PERIOD, V_SIG_PERIOD/2, ccDriver.flags);
        if (ret < 0){
            printk("Error: Failed to set pulse width");
            return -1;
        }
    return ret;
}
