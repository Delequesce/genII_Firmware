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

/* Function Forward Declaration */
static int startDriveSignal(const struct pwm_dt_spec);
static int configure_uart_device(const struct device *dev);
static int uart_write(const struct device *dev, *fmt);
static int configure_ISR(const struct device *dev);
static void uart_rx_interrupt_handler(const struct device *dev);
static void runNewTest();


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
	if (!configure_ISR(uart_dev)){
		return -1;
	}

	return 0; 
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

	if (!uart_configure(dev, &cfg)){
		return -1;
	}
}

/* Writes data to uart */
static int uart_write(const struct device *dev, *fmt){
	
}

/* Configures ISR for interrupt-driven operation */
static int configure_ISR(const struct device *dev){

	uart_irq_callback_user_data_set(dev, &uart_interrupt_handler, dev);

	uart_irq_rx_enable(uart_dev);
	uart_tx_enable(uart_dev);
}


static void uart_interrupt_handler(const struct device *dev){

	/* ACK pending requests */
	if (!uart_irq_update(dev)){
		return;
		}

	/* Check if there is currently data in the tx buffer */
	

	/* Check if uart rx buffer has received a character */
	if (!uart_irq_rx_ready(dev)){
		return;
	}

	/* Set up and read buffer */
	uint8_t rx_data[2];
	
	uart_fifo_read(dev, rx_data, 1);

	/* Decides which subroutine to execute based on sent command. Spawns and cancels appropriate threads */
	switch(rx_data[0]) {
			if (activeState == IDLE){
				case 'C': // Connect to Device
					break;
				case 'S': // Read and alter Test Configuration Structure
					uart_fifo_read(dev, rx_data, 2);
					test_cfg.runTime = rx_data[0];
					test_cfg.collectionInterval = rx_data[1];
					break;
				case 'N': // New Test
					activeState = TESTRUNNING;  
					/* Allocate memory and spawn new thread */
					K_THREAD_STACK_DEFINE(IA_stack_area, IA_STACK_SIZE); 
					ia_tid = k_thread_create(&IA_thread_data, IA_stack_area,
									K_THREAD_STACK_SIZEOF(IA_stack_area),
									testThread_entry_point, 
									&test_cfg, NULL, NULL, 
									IA_THREAD_PRIO, 0, K_NO_WAIT);
					k_yield(); // Should (but might not) be able to yield from an interrupt handler
					break;
				case 'H': // Toggle Heater
					//toggleHeater();
					break;
				case 'B': // Calibrate System
					activeState = CALIBRATING;
					//runCalibration();
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
	return; 
}

static void uartIOThread_entry_point(){

	while(1){
	/* The interrupt handler might be instantly called if flag is set. If not, check periodically if tx or rx uart fifos have data. If so, invoke handler. */
	}
	return;
}


static void testThread_entry_point(const struct *test_cfg, void *unused1, void *unused2){

	/**
	* At 64/62 MHz, samples are collected every 969 nsec. Undersampling pushes the 1 MHz information of interest
	* to F = 32.3 kHz. To center the bins around this frequency with roughly 1 msec, sample time, we need to 
	* sample an integer number of periods (T = 31 usec). This allows exactly 32 periods in 992 usec. At a 
	* sample rate of 64/62 MHz, this translates to 1024 samples per ~1 msec collection. 
	* Two coefficients are calculated from w0 = 2*pi * k/N, where N is number of frequency bins and
	* k is the bin of interest. For 992 usec sample, ~1008 Hx, so the aliased 32.3 kHz bin is bin 32 (if DC is bin 0)
	* Total bin number is same as samplesRead.
	* */

	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);

	/* Disable UART interrupts */
	uart_irq_rx_disable(uart_dev);

	/* Local Variable Initialization */
	uint16_t N_Measurements = (uint16_t)(test_cfg->runTime / test_cfg->collectionInterval);
	uint16_t N_Averages = 100;
	uint16_t samplesPerMeasurement = 1024;
	float w0 = 6283185.30717959;
	const float cos_w0 = 0.9807853;
	const float sin_w0 = 0.1950903:
	float mag2Vr, mag2Z, Z_temp_real, Z_temp_imag;
	float Z_real, Z_imag, C, G = 0;
	uint16_t i, j, n, k;
	float *Vr_ptr;
	float *Ve_ptr;
	float prev_value_Vr, prev_value_Ve;
	float prev_prev_value_Vr, prev_prev_value_Ve;
	float currentValue_Vr, currentValue_Ve;
	float Zfb_real = 1; // These values will come from calibration
	float Zfb_imag = 0;
	char transmit_Byte;


	/* Timing Parameters */
	uint64_t startTime = k_uptime_get();
	uint32_t sleepTime; 
	
	/** 
	* Call generate_Vsig() and ad4002_continuous_read() to start PWM 
	* signals for driving and interrupts 
	* This will automatically trigger SPI reading and DMA transfer to memory 
	* in an interrupt driven manner.
	* Therefore, we just need to periodically disable the PWM signals (or 
	* just the interrupt) so we can process data,
	* service usart interrupts, and return data
	*/

	/* This loop runs each collection for the entire test run time (outer loop) */
	for(i = 0; i < N_Measurements; i++){

		/* This loop runs to obtain repeat measurements over the collection frequency interval */
		for(j = 0; j < N_Averages; j++){
			
			/* Wait ~ 1 msec for read operations to finish (see notes) */
			while(!ad4002_read_finished(ad4002_dev)){
				// Do Nothing
			}
			
			// Prevent system interrupts during processing
			irq_lock();
	
			/** 
			* Read collected data from memory. You will need to get a pointer to the starting and ending memory location
			* for DMA (or total length). From this, you can loop through each address and run the goertzl algorithm to
			* calculate the FFT bin.
			* */
			Vr_ptr = 0x00000000;  // Fill this in with the correct memory location from DMA
			Ve_ptr = 0x00000000;  // Fill this in with the correct memory location from DMA
			prev_value_Vr = 0;
			prev_prev_value_Vr = 0;
			currentValue_Vr = 0;
			prev_value_Ve = 0;
			prev_prev_value_Ve = 0;
			currentValue_Ve = 0;
			
			/* This loop runs for each individual ~ 1 msec collection period */
			for(n = 0; n < samplesPerMeasurement; n++){
				current_value_Vr = (*Vr_ptr) + 2 * cos_w0 * prev_value_Vr - prev_prev_value_Vr;
				prev_prev_value_Vr = prev_value_Vr;
				prev_value_Vr = current_value_Vr; 
				current_value_Ve = (*Ve_ptr) + 2 * cos_w0 * prev_value_Ve - prev_prev_value_Ve;
				prev_prev_value_Ve = prev_value_Ve;
				prev_value_Ve = current_value_Ve; 
				Vr_ptr += 0x4; // Increment 4 Bytes to read next memory location
				Ve_ptr += 0x4; // Increment 4 Bytes to read next memory location
			}
	
			/* Final Step calculates complex FFT bin for each signal */
			Vr_real = cos_w0 * current_value_Vr - prev_value_Vr;
			Vr_imag = sin_w0 * current_value_Vr;
			Ve_real = cos_w0 * current_value_Ve - prev_value_Ve;
			Ve_imag = sin_w0 * current_value_Ve;
	
			/* Finally, compute Z, G, and C */
			mag2Vr = Vr_real*Vr_real + Vr_imag*Vr_imag;
			Z_temp_real = (Ve_real * Vr_real + Ve_imag * Vr_imag)/mag2Vr; 
			Z_temp_imag = (Ve_imag * Vr_real - Ve_real * Vr_imag)/mag2Vr;
			Z_real = Z_temp_real*Zfb_real - Z_temp_imag * Zfb_imag;
			Z_imag = Z_temp_real*Zfb_imag + Z_temp_imag * Z_fb_real;
			mag2Z = Z_real*Z_real + Z_imag * Z_imag;
			G = (G*j + (Z_real/mag2Z))/(j+1); // Moving Average
			C = (C*j + (-Z_imag/mag2Z/w0))/(j+1);
		}
		/* Send data over uart */
		/* Transfer data to uart Tx buffer. When the main thread resumes, the handler will be called
			and write and read operations will commence. Will transfer LSB first */
		for (k = 0; k < N_DATA_BYTES, k++){
			transmit_Byte = (C >> k*8) & 0x000000FF;
			uart_poll_out(uart_dev, transmit_Byte);
		}
		for (k = 0; k < N_DATA_BYTES, k++){
			transmit_Byte = (G >> k*8) & 0x000000FF;
			uart_poll_out(uart_dev, transmit_Byte);
		}
		
		/* Sleep until next collection period */
		sleepTime = (test_cfg->collectionInterval) * 1000 - k_uptime_delta(&startTime);
		k_msleep(sleepTime); 
	}
	activeState = IDLE; 
	return; 
}

static int stopTest(){
	activeState = IDLE;
	k_thread_abort(ia_tid); 
	return 0; 
}


K_THREAD_DEFINE(uartIO, UARTIO_STACK_SIZE, uartIOThread_entry_point, NULL, NULL, NULL, UARTIO_THREAD_PRIORITY, 0, 0);



int main(void)
{
	int ret;
    unsigned int lock_key;
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
