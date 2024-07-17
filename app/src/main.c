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

#define IA_THREAD_PRIO           2    // Adjust as needed
#define UARTIO_THREAD_PRIORITY   5
#define IA_STACK_SIZE            1024
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

	/* Configure and enable UART Interrupt Callback */

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

	/* Insert Measurement and Goertzl loop code */

	// For testing, just write half-word in a loop until buffer is half full
	uint8_t j;
	uint8_t i;
	for(j = 0; j < 1; j++){
		for (i = 0; i < 255; i++){
			uart_poll_out(uart_dev, 'K');
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
