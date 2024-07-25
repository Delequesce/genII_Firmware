#include <stdio.h>
#include <math.h>
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
#include <string.h>

/* Configuration flags */
#define FREE_RUN					0

/* DT NODELABELS */
#define CCDRIVER	        		DT_ALIAS(my_ccdrive)
#define AD4002_INSTANCE_1   		DT_ALIAS(ad4002_ch1)
#define AD4002_INSTANCE_2   		DT_ALIAS(ad4002_ch2)
#define CC_SHDN_LOW         		DT_ALIAS(my_cc_shdn_low)
#define ADC_SHDN_LOW        		DT_ALIAS(my_adc_shdn_low)
#define D0							DT_ALIAS(my_d0)
#define D1							DT_ALIAS(my_d1)

/* Threading Params */
#define IA_THREAD_PRIO           	2    // Adjust as needed
#define UARTIO_THREAD_PRIORITY   	5
#define IA_STACK_SIZE            	2048
#define UARTIO_STACK_SIZE        	1024
#define N_DATA_BYTES             	8 // C (4), G (4) = 8 bytes

/* Measurement params */
#define SAMPLES_PER_COLLECTION  	1024
#define SLEEP_TIME_MS 				100
#define DEFAULT_COLLECTION_INTERVAL	1
#define DEFAULT_RUN_TIME			30
#define DEFAULT_SPOT_FREQUENCY		1000000
#define CONVERT_FREQUENCY			1032258
#define W0							0.19634916

/* Other constants */
#define PI						 	3.141592654
#define V_SIG_PERIOD        		64  /* In clock cycles */
#define N_DATA_BYTES				4 

/* Structs, Unions, and Enums */
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

union float_to_byte {
	float float_variable;
	uint8_t temp_array[N_DATA_BYTES];
};

enum testStates activeState = IDLE;
static struct test_config test_cfg = {
	.runTime = DEFAULT_RUN_TIME,
	.collectionInterval = DEFAULT_COLLECTION_INTERVAL,
};

/* Function Forward Declaration */
static int startDriveSignal(const struct pwm_dt_spec);
static int configure_uart_device(const struct device *dev);
static void uartIOThread_entry_point();
static void testThread_entry_point(const struct test_config* test_cfg, void *unused1, void *unused2);
static int stopTest();

/* Threads */
k_tid_t ia_tid; // IA Measurement Thread
struct k_thread IA_thread_data;
K_THREAD_DEFINE(uartIO, UARTIO_STACK_SIZE, uartIOThread_entry_point, NULL, NULL, NULL, UARTIO_THREAD_PRIORITY, 0, 0);
K_THREAD_STACK_DEFINE(IA_stack_area, IA_STACK_SIZE); 


/* Relevant Device Tree Structures */
const struct device *uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console)); 
static const struct pwm_dt_spec ccDriver = PWM_DT_SPEC_GET(CCDRIVER);
static const struct device* ad4002_master = DEVICE_DT_GET(AD4002_INSTANCE_1);
static const struct device* ad4002_slave = DEVICE_DT_GET(AD4002_INSTANCE_2);
static const struct gpio_dt_spec cc_shdn_low = GPIO_DT_SPEC_GET(CC_SHDN_LOW, gpios);
static const struct gpio_dt_spec adc_shdn_low = GPIO_DT_SPEC_GET(ADC_SHDN_LOW, gpios);
static const struct gpio_dt_spec d0 = GPIO_DT_SPEC_GET(D0, gpios);
static const struct gpio_dt_spec d1 = GPIO_DT_SPEC_GET(D1, gpios);

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

/* Polling based UART handler thread */
static void uartIOThread_entry_point(){

	#if FREE_RUN 
	activeState = TESTRUNNING;  
	/* Allocate memory and spawn new thread */
	ia_tid = k_thread_create(&IA_thread_data, IA_stack_area,
					K_THREAD_STACK_SIZEOF(IA_stack_area),
					testThread_entry_point, 
					&test_cfg, NULL, NULL, 
					IA_THREAD_PRIO, 0, K_NO_WAIT);
	k_yield(); // Yields to newly created thread because priority is higher than UART Thread. UART Thread goes to "Ready" State
	#else
	unsigned char p_char;
	int ret;
	while(1){
		// Polls uart interface 10 times per second for any incoming data
		while (uart_poll_in(uart_dev, &p_char) < 0){
			/* Allow other threads to work */
			//printk("Waiting for input...\n");
			k_msleep(100);
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
	#endif
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

	/* Enable ADC and CC Drive */
	if (gpio_pin_set_dt(&cc_shdn_low, 1) < 0 || gpio_pin_set_dt(&adc_shdn_low, 1) < 0) {
        printk("SHDN pins not properly set");
		return 0;
	}
	if (startDriveSignal(ccDriver) < 0){
        return -1;
    }

	/* Begin Measurement */
	const uint16_t N_Measurements = (uint16_t)(test_cfg->runTime / test_cfg->collectionInterval);
	uint16_t N_Averages = 1;
	uint8_t transmit_Byte;
	uint16_t i, j, n;
	uint16_t buff_fill;
	const float w0 = W0;//2*PI*(1-DEFAULT_SPOT_FREQUENCY/CONVERT_FREQUENCY); 
	const float cos_w0 = cosf(w0);
	const float sin_w0 = sinf(w0);
	float mag2Vr, mag2Z, Z_temp_real, Z_temp_imag;
	float Vr_real, Vr_imag, Ve_real, Ve_imag;
	float Z_real, Z_imag = 0;
	float prev_value_Vr, prev_value_Ve;
	float prev_prev_value_Vr, prev_prev_value_Ve;
	float current_value_Vr, current_value_Ve;
	float Zfb_real = 1; // These values will come from calibration
	float Zfb_imag = 0;
	union float_to_byte C;
	C.float_variable = 0;
	union float_to_byte G;
	G.float_variable = 0;

	unsigned char a_char;

	/* Timing Parameters */
	uint64_t startTime = k_uptime_get();
	uint32_t sleepTime;

	/* Configure TIA SHDN */
    if (!gpio_is_ready_dt(&d0) || !gpio_is_ready_dt(&d1)) {
        printk("TIA Not Selected");
		return 0;
	}
	/* Channel 0 */
    if (gpio_pin_configure_dt(&d0, GPIO_OUTPUT_INACTIVE) < 0 || gpio_pin_configure_dt(&d1, GPIO_OUTPUT_ACTIVE) < 0) {
        printk("TIA Not Selected");
		return 0;
	}

	/* This loop runs each collection for the entire test run time (outer loop) */
	for(i = 0; i < 10; i++){

		/* This loop runs to obtain repeat measurements over the collection frequency interval */
		for(j = 0; j < N_Averages; j++){

			/* Read Data from ADC */
			ad4002_continuous_read(ad4002_master, ad4002_slave, Ve_data, Vr_data, SAMPLES_PER_COLLECTION);
			k_msleep(SLEEP_TIME_MS);
			ad4002_stop_read(ad4002_master);

			/* Copy data to safe memory location */ 
			memcpy(Ve_data_safe, Ve_data, SAMPLES_PER_COLLECTION*2);
			memcpy(Vr_data_safe, Vr_data, SAMPLES_PER_COLLECTION*2);

			/* Goertzl Algorithm Params */
			prev_value_Vr = 0;
			prev_prev_value_Vr = 0;
			prev_value_Ve = 0;
			prev_prev_value_Ve = 0;
			
			/* Run algorithm */
			for(n = 0; n < SAMPLES_PER_COLLECTION; n++){
				current_value_Ve = 3*Ve_data_safe[n]/65535.0 + 2 * cos_w0 * prev_value_Ve - prev_prev_value_Ve;
				prev_prev_value_Ve = prev_value_Ve;
				prev_value_Ve = current_value_Ve; 
			}

			/* Final Step calculates complex FFT bin for each signal */
			Ve_real = (cos_w0 * prev_value_Ve - prev_prev_value_Ve)/SAMPLES_PER_COLLECTION;
			Ve_imag = (sin_w0 * prev_value_Ve)/SAMPLES_PER_COLLECTION;

			/* Finally, compute Z, G, and C */
			mag2Vr = Ve_real*Ve_real + Ve_imag*Ve_imag;
			// Z_temp_real = (Ve_real * Vr_real + Ve_imag * Vr_imag)/mag2Vr; 
			// Z_temp_imag = (Ve_imag * Vr_real - Ve_real * Vr_imag)/mag2Vr;
			// Z_real = Z_temp_real*Zfb_real - Z_temp_imag * Zfb_imag;
			// Z_imag = Z_temp_real*Zfb_imag + Z_temp_imag * Z_fb_real;
			// mag2Z = Z_real*Z_real + Z_imag * Z_imag;
			// G.float_variable = (G.float_variable*j + (Z_real/mag2Z))/(j+1); // Moving Average
			// C.float_variable = (C.float_variable*j + (-Z_imag/mag2Z/w0))/(j+1);

			// Send newline to stop UI loop
			C.float_variable = mag2Vr;

			for(n = N_DATA_BYTES; n > 0; n--){
				transmit_Byte = C.temp_array[n-1];
				uart_poll_out(uart_dev, transmit_Byte);
			}

			//uart_poll_out(uart_dev, '\n');
		}

		/* Send data over uart */
		/* Transfer data to uart Tx buffer. When the main thread resumes, the handler will be called
			and write and read operations will commence. Will transfer MSB first */
		// C.float_variable = Vr_real;
		// G.float_variable = Vr_imag;
		// for (n = N_DATA_BYTES; n > 0; n--){
		// 	uart_poll_out(uart_dev, C.temp_array[n-1]);
		// }
		// for (n = N_DATA_BYTES; n > 0; n--){
		// 	uart_poll_out(uart_dev, G.temp_array[n-1]);
		// }

		/* Sleep until next collection period */
		sleepTime = (test_cfg->collectionInterval) * 1000 - k_uptime_delta(&startTime);
		sleepTime =  1000;
		k_msleep(sleepTime);
	}
	/* Go back to UART (thread is automatically terminated) */
	for(i = 0; i<3; i++){
		for(n = 0; n < 4; n++){
			transmit_Byte = n;
			uart_poll_out(uart_dev, transmit_Byte); // Ends collection on UI side
		}
	}
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