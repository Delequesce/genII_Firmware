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
#define AD4002_INSTANCE_2   DT_ALIAS(ad4002_ch2)
#define V_SIG_PERIOD        64  /* In clock cycles */

#define SAMPLES_PER_COLLECTION  1024
#define SLEEP_TIME_MS 100




/* Function Forward Declaration */
static int startDriveSignal(const struct pwm_dt_spec);

bool data_valid;

int main(void)
{
	int ret;
    unsigned int lock_key;
    data_valid = false;
    static uint16_t Ve_data[SAMPLES_PER_COLLECTION]; // Memory allocation for ADC RX Data  
    static uint16_t Vr_data[SAMPLES_PER_COLLECTION]; // Memory allocation for ADC RX Data  

    /* Obtain Relevant Device Tree Structures */
    static const struct pwm_dt_spec ccDriver = PWM_DT_SPEC_GET(CCDRIVER);
    static const struct device* ad4002_master = DEVICE_DT_GET(AD4002_INSTANCE_1);
    static const struct device* ad4002_slave = DEVICE_DT_GET(AD4002_INSTANCE_2);

    if (startDriveSignal(ccDriver) < 0){
        return -1;
    }
    uint32_t n;
    for(n = 0; n < SAMPLES_PER_COLLECTION; n++){
        *(Ve_data + n) = 1;
        *(Vr_data + n) = 1;
    }
    
    // For manual reads
    uint16_t* rx_data;
    
	while (1) {
        /* Begin Interrupt Driven ADC Read Operation */
        ad4002_continuous_read(ad4002_master, ad4002_slave, Vr_data, Ve_data, SAMPLES_PER_COLLECTION); // Slave start 
        //ad4002_continuous_read(ad4002_device_1, Vr_data, SAMPLES_PER_COLLECTION); // Master start

        /* Waits for enough time and then stops the read */
        
        k_msleep(SLEEP_TIME_MS);
        //lock_key = irq_lock(); // Prevent system interrupts during processing
        rx_data = ad4002_stop_read(ad4002_master);
        /*for(n = 0; n < SAMPLES_PER_COLLECTION; n++){
            if (n % 100 == 0){
                printk("Ve %d: %d\n", n, *(Ve_data + n));
                printk("Vr %d: %d\n", n, *(Vr_data + n));
            }
        }*/
        k_msleep(1000);
        //irq_unlock(lock_key);

        for(n = 0; n < SAMPLES_PER_COLLECTION; n+=100){
            printk("Ve %d: %d\n", n, *(Ve_data + n));
            printk("Vr %d: %d\n", n, *(Vr_data + n));
            //printk("Rx %d: %d\n", n, *(rx_data + n));
        }
	 }
	return 0;
}

static int startDriveSignal(const struct pwm_dt_spec ccDriver){

    int ret;
    // ClotChip Drive Signal 
    ret = pwm_set_cycles(ccDriver.dev, ccDriver.channel, V_SIG_PERIOD, V_SIG_PERIOD/2, ccDriver.flags);
        if (ret < 0){
            printk("Error: Failed to set pulse width");
            return -1;
        }
    return ret;
}
