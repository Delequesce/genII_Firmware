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
#define CC_SHDN_LOW         DT_ALIAS(my_cc_shdn_low)
#define ADC_SHDN_LOW        DT_ALIAS(my_adc_shdn_low)
#define V_SIG_PERIOD        64  /* In clock cycles */

#define SAMPLES_PER_COLLECTION  1024
#define SLEEP_TIME_MS 100

#define IA_THREAD_PRIO           2    // Adjust as needed
#define UARTIO_THREAD_PRIORITY   5
#define IA_STACK_SIZE            2048
#define UARTIO_STACK_SIZE        1024
#define N_DATA_BYTES             8 // C (4), G (4) = 8 bytes


enum testStates {
	IDLE,
	TESTRUNNING,
	CALIBRATING,
	EQC
};

struct test_config{
	uint16_t runTime; // Test run time in seconds
	uint16_t collectionInterval; // Interval in seconds between measurements
};


// Get UART node from Device Tree
const struct device *uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console)); 

// Other Global Variables
enum testStates activeState = IDLE;
struct k_thread IA_thread_data;
struct test_config test_cfg = {
	.runTime = 1800,
	.collectionInterval = 1,
};

/* Threads */
k_tid_t ia_tid; // IA Measurement Thread

/* Function Forward Declaration */
static int startDriveSignal(const struct pwm_dt_spec);
static int configure_uart_device(const struct device *dev);
static int uart_write(const struct device *dev, void *fmt);
static int configure_ISR(const struct device *dev);
static void uart_rx_interrupt_handler(const struct device *dev);
static void uartIOThread_entry_point();
static void testThread_entry_point(const struct test_config* test_cfg, void *unused1, void *unused2);
static int stopTest();

/* Threads */
K_THREAD_DEFINE(uartIO, UARTIO_STACK_SIZE, uartIOThread_entry_point, NULL, NULL, NULL, UARTIO_THREAD_PRIORITY, 0, 0);
K_THREAD_STACK_DEFINE(IA_stack_area, IA_STACK_SIZE); 


/* Relevant Device Tree Structures */
static const struct pwm_dt_spec ccDriver = PWM_DT_SPEC_GET(CCDRIVER);
static const struct device* ad4002_master = DEVICE_DT_GET(AD4002_INSTANCE_1);
static const struct device* ad4002_slave = DEVICE_DT_GET(AD4002_INSTANCE_2);
static const struct gpio_dt_spec cc_shdn_low = GPIO_DT_SPEC_GET(CC_SHDN_LOW, gpios);
static const struct gpio_dt_spec adc_shdn_low = GPIO_DT_SPEC_GET(ADC_SHDN_LOW, gpios);


/* Sets up devices */
int main(){

	/* Check Device Readiness */

	if (!device_is_ready(uart_dev)){
		return -1;
	}

	/* Configure Device Params */
	if (!configure_uart_device(uart_dev)){
		return -1;
	}

	/* Setup ADC Power and CC Drive Signal */
    if (!gpio_is_ready_dt(&cc_shdn_low) || !gpio_is_ready_dt(&adc_shdn_low)) {
        printk("SHDN pins not ready");
		return 0;
	}
    if (gpio_pin_configure_dt(&cc_shdn_low, GPIO_OUTPUT_ACTIVE) < 0 || gpio_pin_configure_dt(&adc_shdn_low, GPIO_OUTPUT_ACTIVE) < 0) {
        printk("SHDN pins not properly configured");
		return 0;
	}

	return 0; // Scheduler invokes highest priority ready thread, which is uartIOThread (goes to entry point)
}

/* Configures UART communication parameters */
static int configure_uart_device(const struct device *dev){

	struct uart_config cfg = {
		.baudrate = 115200,
		.parity = UART_CFG_PARITY_NONE,
		.stop_bits = UART_CFG_STOP_BITS_1,
		.data_bits = UART_CFG_DATA_BITS_8,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
	};

	if (uart_configure(dev, &cfg) < 0){
		return -1;
	}
	return 0;
}

/* Writes data to uart */
static int uart_write(const struct device *dev, void *fmt){
	return 0; 
}

/* Polling based UART handler thread */
static void uartIOThread_entry_point(){

	unsigned char p_char;
	int ret;
	while(1){
		// Polls uart interface once per second for any incoming data
		while (uart_poll_in(uart_dev, &p_char) < 0){
			/* Allow other threads to work */
			//printk("Waiting for input...\n");
			k_msleep(1000);
		}

		/* Decides which subroutine to execute based on sent command. Spawns and cancels appropriate threads */
		switch(p_char) {
			if (activeState == IDLE){
				case 'C': // Connect to Device
					uart_poll_out(uart_dev, 'K');
					break;
				case 'S': // Read and alter Test Configuration Structure
					uint16_t* p_u16;
					uart_poll_in_u16(uart_dev, p_u16);
					test_cfg.runTime = (*p_u16 & 0x00FF);
					test_cfg.collectionInterval = (*p_u16 & 0xFF00);
					break;
				case 'N': // New Test
					activeState = TESTRUNNING;  
					/* Allocate memory and spawn new thread */
					ia_tid = k_thread_create(&IA_thread_data, IA_stack_area,
									K_THREAD_STACK_SIZEOF(IA_stack_area),
									testThread_entry_point, 
									&test_cfg, NULL, NULL, 
									IA_THREAD_PRIO, 0, K_NO_WAIT);
					k_yield(); // Yields to newly created thread because priority is higher than UART Thread. UART Thread goes to "Ready" State
					break;
				case 'H': // Toggle Heater
					//toggleHeater();
					break;
				case 'B': // Calibrate System
					activeState = CALIBRATING;
					uart_poll_out(uart_dev, 'K');
					//runCalibration();
					activeState = IDLE;
					break;
				case 'Q': // Run EQC
					activeState = EQC;
					//runEQC();
					break;
				default:
					printk("Unknown Command\n");
			}
			else{
				case 'X': // Stop Test
					if (stopTest() < 0){
						break;
					}
					activeState = IDLE; 
					// Cancel currently running test thread
					// ...
					break;
			}
		}
	}
	return;
}

/* Performs measurements */
static void testThread_entry_point(const struct test_config* test_cfg, void *unused1, void *unused2){

	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);

	/* Data Buffer Initialization */
	static uint16_t Ve_data[SAMPLES_PER_COLLECTION]; // Memory allocation for ADC RX Data  
    static uint16_t Vr_data[SAMPLES_PER_COLLECTION]; // Memory allocation for ADC RX Data 
	static uint16_t Ve_data_safe[SAMPLES_PER_COLLECTION]; // Memory allocation for ADC RX Data  
    static uint16_t Vr_data_safe[SAMPLES_PER_COLLECTION]; // Memory allocation for ADC RX Data 
	for(uint32_t n = 0; n < SAMPLES_PER_COLLECTION; n++){
        *(Ve_data + n) = 1;
        *(Vr_data + n) = 1;
    } 
	// For manual reads
    uint16_t* rx_data;

	/* Enable ADC and CC Drive */
	if (gpio_pin_set_dt(&cc_shdn_low, 1) < 0 || gpio_pin_set_dt(&adc_shdn_low, 1) < 0) {
        printk("SHDN pins not properly set");
		return 0;
	}
	if (startDriveSignal(ccDriver) < 0){
        return -1;
    }

	/* Begin Measurement */
	uint8_t transmit_Byte;
	uint32_t i, j;
	for(i = 0; i < 1; i++){
		/* Read Data from ADC */
		ad4002_continuous_read(ad4002_master, ad4002_slave, Ve_data, Vr_data, SAMPLES_PER_COLLECTION);
		k_msleep(SLEEP_TIME_MS);
		rx_data = ad4002_stop_read(ad4002_master);

		/* Process Data (Goertzl + Permittivity Calculations)*/
		// To Do ...

		// Copy data to safe memory location
		memcpy(Ve_data_safe, Ve_data, SAMPLES_PER_COLLECTION*2);
		memcpy(Vr_data_safe, Vr_data, SAMPLES_PER_COLLECTION*2);

		/* Send data to UI */ 
		for (j = 0; j < SAMPLES_PER_COLLECTION; j++){
			// High Byte
			transmit_Byte = (Vr_data_safe[j] >> 8) & 0xFF;
			uart_poll_out(uart_dev, transmit_Byte);
			//k_busy_wait(1000);
			// Low Byte
			transmit_Byte = Vr_data_safe[j] & 0xFF;
			uart_poll_out(uart_dev, transmit_Byte);
			//k_busy_wait(1000);
		}
		k_msleep(1000);
	}

	// Send newline to stop UI loop
	uart_poll_out(uart_dev, '\n');

	// Go back to UART (thread is automatically terminated)
	activeState = IDLE;
	return; 
}

static int stopTest(){
	activeState = IDLE;
	k_thread_abort(ia_tid); 
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