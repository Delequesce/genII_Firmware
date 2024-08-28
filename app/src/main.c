#include <stdio.h>
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/sys/printk.h>
#include <app/drivers/ad4002.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/pwm.h>
#include <stm32_ll_tim.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/adc.h>
#include <string.h>
#include <app/main.h>

/* Default States */
#if FREE_RUN
enum testStates activeState = FREERUNNING;
#else
enum testStates activeState = IDLE;
#endif
enum heaterStates heaterState = NOT_HEATING; 

static struct test_config test_cfg = {
	.runTime = DEFAULT_RUN_TIME,
	.collectionInterval = DEFAULT_COLLECTION_INTERVAL,
	.incubationTemp = DEFAULT_INCUBATION_TEMP,
};

/* Load with default values */
static struct calibration_data calib = {
	.Zfb_real = DEFAULT_ZFB_REAL,
	.Zfb_imag = DEFAULT_ZFB_IMAG,
};


/* Threads */
k_tid_t ia_tid; // IA Measurement Thread
struct k_thread IA_thread_data;
struct k_thread heater_thread_data;
#if HEATER
K_THREAD_DEFINE(heater, HEATER_STACK_SIZE, heaterThread_entry_point, NULL, NULL, NULL, HEATER_THREAD_PRIORITY, 0, 0);
#endif
#if FREE_RUN
K_THREAD_DEFINE(ia, IA_STACK_SIZE, testThread_entry_point, &test_cfg, NULL, NULL, IA_THREAD_PRIORITY, 0, 0);
#else
K_THREAD_DEFINE(uartIO, UARTIO_STACK_SIZE, uartIOThread_entry_point, NULL, NULL, NULL, UARTIO_THREAD_PRIORITY, 0, 0);
K_THREAD_STACK_DEFINE(IA_stack_area, IA_STACK_SIZE); 
#endif

/* Message and work queue */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 4, 1); // Message queue can handle 10 items of size MSG_SIZE (bytes), aligned to 1 byte boundary. 

/* Relevant Device Tree Structures */
const struct device *uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
static const struct device *flash_device = DEVICE_DT_GET(DT_CHOSEN(zephyr_flash_controller));
static const struct pwm_dt_spec ccDriver = PWM_DT_SPEC_GET(CCDRIVER);
static const struct pwm_dt_spec heaterPwm = PWM_DT_SPEC_GET(HEATERPWM);
static const struct device* ad4002_master = DEVICE_DT_GET(AD4002_INSTANCE_1);
static const struct device* ad4002_slave = DEVICE_DT_GET(AD4002_INSTANCE_2);
static const struct gpio_dt_spec cc_shdn_low = GPIO_DT_SPEC_GET(CC_SHDN_LOW, gpios);
static const struct gpio_dt_spec adc_shdn_low = GPIO_DT_SPEC_GET(ADC_SHDN_LOW, gpios);
static const struct gpio_dt_spec d0 = GPIO_DT_SPEC_GET(D0, gpios);
static const struct gpio_dt_spec d1 = GPIO_DT_SPEC_GET(D1, gpios);
static const struct gpio_dt_spec heater_en = GPIO_DT_SPEC_GET(HEATER_EN, gpios);

#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
	!DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified"
#endif

static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
			     DT_SPEC_AND_COMMA)
};

/* UART ISR params */
static char rx_buf[MSG_SIZE];
static int rx_buf_pos = 0;

//static const struct adc_dt_spec adc_channels[] = {ADC_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), vth2)};


/* ISR strictly needs to read data from FIFO and store into queue for message handler workthread */
void uart_rx_isr(const struct device *dev, void *user_data){
	
	uint8_t c;
	int ret;
	/* ACK pending requests */
	if (!uart_irq_update(uart_dev)) {
		return;
	}

	/* Check if uart rx buffer has received a character */
	ret = uart_irq_rx_ready(uart_dev);
	if (!ret) {
		return;
	}

	/* Read until FIFO is empty */
	while (uart_fifo_read(uart_dev, &c, 1) == 1) {
		if ((c == '\n' || c == '\r') && rx_buf_pos > 0) {
			/* terminate string */
			rx_buf[rx_buf_pos] = '\0';

			/* if queue is full, message is silently dropped */
			k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);

			/* reset the buffer (it was copied to the msgq) */
			rx_buf_pos = 0;
		} else if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
			rx_buf[rx_buf_pos++] = c;
		}
		/* else: characters beyond buffer size are dropped */
	}

}


/* Sets up devices */
int main(){

	int ret;
	/* Check Device Readiness */
	if (!device_is_ready(uart_dev)){
		printk("Uart not ready\n");
		return -1;
	}

	/* Configure Device Params */
	ret = configure_uart_device(uart_dev);
	if (ret < 0){
		printk("Uart failed to initialize\n");
		return -1;
	}

	/* Setup Heater power, ADC Power and CC Drive Signal */
    if (!gpio_is_ready_dt(&cc_shdn_low) || !gpio_is_ready_dt(&adc_shdn_low)) {
        printk("SHDN pins not ready");
		return 0;
	}
    if (gpio_pin_configure_dt(&cc_shdn_low, GPIO_OUTPUT_ACTIVE) < 0 || gpio_pin_configure_dt(&adc_shdn_low, GPIO_OUTPUT_ACTIVE) < 0) {
        printk("SHDN pins not properly configured");
		return 0;
	}
	if (!gpio_is_ready_dt(&heater_en)) {
        printk("Heater pins not ready");
		return 0;
	}
	if (gpio_pin_configure_dt(&heater_en, GPIO_OUTPUT_ACTIVE) < 0) {
        printk("Heater pins not configured");
		return 0;
	}
	
	printk("Setup Completed\n");

	k_yield();

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
	
	/* Set ISR and enable interrupts */
	int ret = uart_irq_callback_user_data_set(uart_dev, uart_rx_isr, NULL);
	if (ret < 0) {
		if (ret == -ENOTSUP) {
			printk("Interrupt-driven UART API support not enabled\n");
		} else if (ret == -ENOSYS) {
			printk("UART device does not support interrupt-driven API\n");
		} else {
			printk("Error setting UART callback: %d\n", ret);
		}
		return 0;
	}

	uart_irq_rx_enable(uart_dev);
	
	return 0;


}

/* Polling based UART handler thread */
static void uartIOThread_entry_point(){

	while(1){
		/* Check messagequeue */
		while (k_msgq_num_used_get(&uart_msgq)) {
			unsigned char p_char[MSG_SIZE]; 
        	k_msgq_get(&uart_msgq, &p_char, K_NO_WAIT);

			/* Decides how to process data based on control character */
			switch(p_char[0]) {
				if (activeState == IDLE){
					case 'C': // Connect to Device
						uart_poll_out(uart_dev, 'K');
						deviceConnected = true;
						break;
					case 'S': // Alter Test Configuration Structure, convert from ascii encoding
						test_cfg.runTime = 1000 * p_char[1] + 100 * p_char[2] + 10 * p_char[3] + p_char[4] - 53328;
						test_cfg.collectionInterval = p_char[5]-48;
						test_cfg.incubationTemp = 10 * p_char[6] + p_char[7] - 528;
						// Respond positively
						uart_poll_out(uart_dev, 'K');
						break;
					case 'N': // New Test
						activeState = TESTRUNNING;
						/* Allocate memory and spawn new thread */
						ia_tid = k_thread_create(&IA_thread_data, IA_stack_area,
										K_THREAD_STACK_SIZEOF(IA_stack_area),
										testThread_entry_point, 
										&test_cfg, NULL, NULL, 
										IA_THREAD_PRIORITY, 0, K_NO_WAIT);
						break;
					case 'H': // Toggle Heater
						/* Toggle global parameter for heater thread to observe */
						heaterState = (heaterState + 1) % 2;
						heater_errI = 0; // Reset Integral counter
						break;
					case 'B': // Calibrate System
						activeState = CALIBRATING;
						struct test_config calib_cfg = {
							.runTime = DEFAULT_CALIBRATION_TIME,
							.collectionInterval = DEFAULT_COLLECTION_INTERVAL,
						};
						ia_tid = k_thread_create(&IA_thread_data, IA_stack_area,
										K_THREAD_STACK_SIZEOF(IA_stack_area),
										testThread_entry_point, 
										&calib_cfg, NULL, NULL, 
										IA_THREAD_PRIORITY, 0, K_NO_WAIT);
						break;
					case 'Q': // Run EQC
						activeState = EQC;
						//runEQC();
						break;
					case 'F': // Free Run
						activeState = FREERUNNING;
						/* Allocate memory and spawn new thread */
						ia_tid = k_thread_create(&IA_thread_data, IA_stack_area,
										K_THREAD_STACK_SIZEOF(IA_stack_area),
										testThread_entry_point, 
										&test_cfg, NULL, NULL, 
										IA_THREAD_PRIORITY, 0, K_NO_WAIT);
						break;
				}
				else{
					case 'X': // Stop Test
						if (stopTest() < 0){
							break;
						}
						if (activeState == TESTRUNNING){
							uart_poll_out(uart_dev, 'X');
						}
						activeState = IDLE; 
						k_thread_abort(ia_tid);
						break;
				}
			}
		}
		/* Yield to newly spawned thread or to heater thread */
		k_yield();
	}
	return;
}

static void heaterThread_entry_point(void *unused1, void *unused2, void *unused3){

	/* Casts unused params to void to avoid compiler warnings */
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	ARG_UNUSED(unused3);

	/* Enable Heater power */
	if (gpio_pin_set_dt(&heater_en, 1) < 0) {
		return 0;
	}

	/* Configure */
	uint32_t count = 0;
	uint32_t pulse_cycles = 0;

	/* Buffer where samples will be written */
	uint16_t buf;
	struct adc_sequence sequence = {
		.buffer = &buf,
		/* buffer size in bytes, not number of samples */
		.buffer_size = sizeof(buf),
		.calibrate = false,
	};
	
	/* Configure channels and sequence individually prior to sampling. */
	for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
		if (!adc_is_ready_dt(&adc_channels[i])) {
			printk("ADC controller device %s not ready\n", adc_channels[i].dev->name);
			return -1;
		}

		if (adc_channel_setup_dt(&adc_channels[i]) < 0) {
			printk("Could not setup channel #%d\n", i);
			return -1;
		}
	}
	int32_t val_mv;
	float heater_errP;
	float channel_temps[NUM_THERMISTORS];

	while(1){

		/* Read current temperature values from ADC */
		for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
			if (adc_sequence_init_dt(&adc_channels[i], &sequence) < 0) {
				//LOG_ERR("Could not initalize sequnce");
				return 0;
			}
			adc_read_dt(&adc_channels[i], &sequence);
			val_mv = (int32_t)buf;
			adc_raw_to_millivolts_dt(&adc_channels[i], &val_mv);
			channel_temps[i] = adc_mv_to_temperature(val_mv);
			//printk("Channel %d reading: %d\n", i, val_mv[i]);
		}
		/* Make sure no outliers and find average */
		float tempAvg = channel_temps[0]; 
		for (size_t i = 1U; i < NUM_THERMISTORS; i++){
			tempAvg = (channel_temps[i] + tempAvg * i)/(i+1);
		}
		for (size_t i = 0U; i < NUM_THERMISTORS; i++){
			if (fabs(channel_temps[i] - tempAvg) > TEMP_DIFF_THRESH){
				printk("ETemperatures on board are spatially uneven\n");
			}
		}
		/* Send temperature reading to GUI */
		if(deviceConnected){
			uart_write_32f(&tempAvg, 1, 'T');
		}

		// To Do...
		if(heaterState == HEATING){
			
			/* Find proportinal and integral error */
			heater_errP = 37.0-tempAvg;
			heater_errI = heater_errI + heater_errP;
			
			pulse_cycles = (uint32_t)(K_P * heater_errP + K_I * heater_errI);

			// Update duty cycle using zephyr driver
			pulse_cycles = 32;
			
			if (pwm_set_cycles(heaterPwm.dev, heaterPwm.channel, V_SIG_PERIOD, pulse_cycles, heaterPwm.flags) < 0){
				printk("Error: Failed to set heater pulse");
				return -1;
			}

		}

		/* Schedule new reading every second and let other threads run */
		k_msleep(2000);
	}
	return;
}

/* Performs measurements */
static void testThread_entry_point(const struct test_config* test_cfg, void *unused1, void *unused2){

	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	//printk("EPlaceholder\n");

	/** Get Current Calibration Data from Flash and read into calibration structure 
	 * If normal test is running, load the most recent calibration. 
	 * If test is for calibration, use default values for easy calibration. 
	*/
	if (activeState == TESTRUNNING){
		flash_read(flash_device, PAGE200, &calib, 8);
	}

	/* Data Buffer Initialization */
	static uint16_t Ve_data[SAMPLES_PER_COLLECTION]; // Memory allocation for ADC RX Data  
    static uint16_t Vr_data[SAMPLES_PER_COLLECTION]; // Memory allocation for ADC RX Data 
	static uint16_t Ve_data_safe[SAMPLES_PER_COLLECTION]; // Memory allocation for ADC RX Data  
    static uint16_t Vr_data_safe[SAMPLES_PER_COLLECTION]; // Memory allocation for ADC RX Data 
	/*for(uint32_t n = 0; n < SAMPLES_PER_COLLECTION; n++){
        *(Ve_data + n) = 1;
        *(Vr_data + n) = 1;
    } */

	/* Enable ADC and CC Drive */
	if (gpio_pin_set_dt(&cc_shdn_low, 1) < 0 || gpio_pin_set_dt(&adc_shdn_low, 1) < 0) {
        printk("SHDN pins not properly set");
		return 0;
	}
	if (pwm_set_cycles(ccDriver.dev, ccDriver.channel, V_SIG_PERIOD, V_SIG_PERIOD/2, ccDriver.flags) < 0){
		printk("Error: Failed to setup CC Drive");
		return -1;
	}

	/* Begin Measurement */
	const uint16_t N_Measurements = (uint16_t)(test_cfg->runTime / test_cfg->collectionInterval);
	uint16_t N_Averages = 100;
	uint16_t i, j;
	uint16_t n;
	const float w0 = W0;//2*PI*(1-DEFAULT_SPOT_FREQUENCY/CONVERT_FREQUENCY); 
	const float cos_w0 = cosf(w0);
	const float sin_w0 = sinf(w0);
	float mag2Vr, mag2Ve, mag2Z, Z_temp_real, Z_temp_imag;
	float Vr_real, Vr_imag, Ve_real, Ve_imag;
	float Z_real, Z_imag = 0;
	float Z_real_mean, Z_imag_mean = 0;
	float prev_value_Vr, prev_value_Ve;
	float prev_prev_value_Vr, prev_prev_value_Ve;
	float current_value_Vr, current_value_Ve;
	struct impedance_data impDat = {.C = 0, .G = 0};

	unsigned char a_char;

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

	/* Sets up necessary peripherals (DMA, SPI, Timers) for reads. */
	ad4002_init_read(ad4002_master, ad4002_slave, Ve_data, Vr_data, SAMPLES_PER_COLLECTION);
	ad4002_irq_callback_set(ad4002_master, &dma_tcie_callback);

	if(activeState == FREERUNNING){
		int8_t N_chunks = SAMPLES_PER_COLLECTION/32;
		while(1){
			
			/* Within loop, start read, wait for interrupt, then processing */
			ad4002_start_read(ad4002_master, SAMPLES_PER_COLLECTION);
			k_msleep(SLEEP_TIME_MS); // Thread sleeps until DMA callback is triggered
			// Processing Code
			//...
		#if FREE_RUN
			k_msleep(1000);
		}
		#else

			/* Copy data to safe memory location */ 
			memcpy(Ve_data_safe, Ve_data, SAMPLES_PER_COLLECTION*2);
			memcpy(Vr_data_safe, Vr_data, SAMPLES_PER_COLLECTION*2);

			/* Loop through and send all data to UI */
			char buffer[9];
			float tempFloat;
			unsigned char s_char;
			uint8_t n, k;
			/* We have to chunk the data to avoid buffer overflow */
			
			for(j = 0; j < N_chunks; j++){
				s_char = 0;
				uart_poll_out(uart_dev, 'F');
				for(i = 32 * j; i < 32*(j+1); i++){
					tempFloat = 3*Ve_data_safe[i]/65535.0f;
					n = snprintf(buffer, 9, "%f", tempFloat);
					for(k = 0; k < n; k++){
						uart_poll_out(uart_dev, buffer[k]);
					}
					uart_poll_out(uart_dev, '!');
					tempFloat = 3*Vr_data_safe[i]/65535.0f;
					n = snprintf(buffer, 9, "%f", tempFloat);
					for(k = 0; k < n; k++){
						uart_poll_out(uart_dev, buffer[k]);
					}
					uart_poll_out(uart_dev, '!');
				}
				uart_poll_out(uart_dev, '\n');
				/* Wait for response from UI before sending more data */
				/*while (s_char != 'K'){
					uart_poll_in(uart_dev, &s_char);
				}*/
			}
			
			k_msleep(1000);
		}
		#endif
	}
	else{

	#if FREE_RUN
	}
	#else
	int64_t sleepTime, timeStamp; // Timing params for measuring speed

	/* Timing Parameters */
	int64_t startTime = k_uptime_get();

	/* This loop runs each collection for the entire test run time (outer loop) */
	for(i = 0; i < 30; i++){

		/* This loop runs to obtain repeat measurements over the collection frequency interval */
		//t1 = k_uptime_get();
		ad4002_start_read(ad4002_master, SAMPLES_PER_COLLECTION);
		k_msleep(2); // Thread sleeps until DMA callback is triggered

		for(j = 0; j < N_Averages; j++){

			/* Read Data from ADC */
			ad4002_start_read(ad4002_master, SAMPLES_PER_COLLECTION);
			k_msleep(2); // Thread sleeps until DMA callback is triggered

			//t2 = k_uptime_get();
			/* Copy data to safe memory location */ 
			memcpy(Ve_data_safe, Ve_data, SAMPLES_PER_COLLECTION*2);
			memcpy(Vr_data_safe, Vr_data, SAMPLES_PER_COLLECTION*2);

			/* Goertzl Algorithm Params */
			prev_value_Vr = 0;
			prev_prev_value_Vr = 0;
			prev_value_Ve = 0;
			prev_prev_value_Ve = 0;
			
			/* Run algorithm */
			//t3 = k_uptime_get();
			for(n = SAMPLES_PER_COLLECTION-N_FFT; n < SAMPLES_PER_COLLECTION; n++){
				current_value_Ve = Ve_data_safe[n] + 2 * cos_w0 * prev_value_Ve - prev_prev_value_Ve;
				prev_prev_value_Ve = prev_value_Ve;
				prev_value_Ve = current_value_Ve; 

				current_value_Vr = Vr_data_safe[n] + 2 * cos_w0 * prev_value_Vr - prev_prev_value_Vr;
				prev_prev_value_Vr = prev_value_Vr;
				prev_value_Vr = current_value_Vr; 
			}
			//t4 = k_uptime_get();
			/* Final Step calculates complex FFT bin for each signal */
			Ve_real = 3*(cos_w0 * prev_value_Ve - prev_prev_value_Ve)/N_FFT/65535.0f;
			Ve_imag = 3*(sin_w0 * prev_value_Ve)/N_FFT/65535.0f;

			Vr_real = 3*(cos_w0 * prev_value_Vr - prev_prev_value_Vr)/N_FFT/65535.0f;
			Vr_imag = 3*(sin_w0 * prev_value_Vr)/N_FFT/65535.0f;

			/* Finally, compute Z, G, and C */
			Z_temp_real = COMPLEX_DIVIDE_REAL(Ve_real, Ve_imag, Vr_real, Vr_imag);
			Z_temp_imag = COMPLEX_DIVIDE_IMAG(Ve_real, Ve_imag, Vr_real, Vr_imag);
			Z_real = (Z_real*j + COMPLEX_MULTIPLY_REAL(Z_temp_real, Z_temp_imag, calib.Zfb_real, calib.Zfb_imag))/(j+1); // Moving Average
			Z_imag = (Z_imag*j + COMPLEX_MULTIPLY_IMAG(Z_temp_real, Z_temp_imag, calib.Zfb_real, calib.Zfb_imag))/(j+1);

		}
		/* Update calibration moving average */
		if (activeState == CALIBRATING){
			Z_real_mean = (Z_real_mean * i + Z_real)/(i+1);
			Z_imag_mean = (Z_imag_mean * i + Z_imag)/(i+1);
			printk("EZ_real: %0.4f\n", Z_real_mean);
			printk("EZ_imag: %0.4f\n", Z_imag_mean);
		}
		else{
			/* Final Calculation */
			mag2Z = Z_real*Z_real + Z_imag*Z_imag;
			impDat.G = 1000 * Z_real/mag2Z;				// Result is in mS
			impDat.C = 159154.943091895f * Z_imag/mag2Z; // magic number is 1e12 / (2*pi*1e6). Result is in pF

			/* Collection timestamp */
			timeStamp = k_uptime_get();

			/* Send data over uart */
			uart_write_32f(&impDat, 2, 'D');
			//printk("E%lli\n", timeStamp);
		}

		/* Sleep until next collection period */
		sleepTime = (test_cfg->collectionInterval) * 1000*(i+1) - timeStamp+startTime;
		k_msleep(sleepTime);
		//t2 = k_uptime_get();
		//printk("TTotal loop time: %lli\n", t2-t1);
		//k_msleep(1000);
	}

	/* Perform additional calibration steps, if necessary */
	if(activeState == CALIBRATING){
		const float test_Z_real = 180.0;
		const float test_Z_imag = 0.0;

		calib.Zfb_real = COMPLEX_DIVIDE_REAL(test_Z_real, test_Z_imag, Z_real_mean, Z_imag_mean);
		calib.Zfb_imag = COMPLEX_DIVIDE_IMAG(test_Z_real, test_Z_imag, Z_real_mean, Z_imag_mean);

		/* Send new values over uart*/
		uart_write_32f(&calib, 2, 'C');

		/* Write to flash memory */
		//flash_write_protection_set(flash_device, false);
		flash_erase(flash_device, PAGE200, 8);
		flash_write(flash_device, PAGE200, &calib, 8);
		//flash_write_protection_set(flash_device, true);
		activeState = IDLE;
		return;
	}
	activeState = IDLE;
	printk("X\n");
	return; 
	}
#endif
}

/* General write function that takes in a pointer to a 32b data, the number of data, and an id code */
static void uart_write_32f(float* data, uint8_t numData, char messageCode){
	
	char buffer[9];

	uint8_t n, i, k = 0;

	uart_poll_out(uart_dev, messageCode);
	for (i = 0; i < numData; i++){
		n = snprintf(buffer, 9, "%f", *data);
		for(k = 0; k < n-1; k++){
			uart_poll_out(uart_dev, buffer[k]);
		}
		data++;
		if (i+1 < numData){
			uart_poll_out(uart_dev, '!');
		}
		else{
			uart_poll_out(uart_dev, 0xA);
		}
	}
	return;
}

static int stopTest(){
	activeState = IDLE;
	k_thread_abort(ia_tid); 
	return 0; 
}

static float adc_mv_to_temperature(int32_t val_mv){
	float m_temp = 0;
	float b_temp = 0;
	return (float)(val_mv * m_temp + b_temp); 
}

/* Wake up sleeping main thread following full dma transfer to resume normal processing */
static void dma_tcie_callback(){

	k_wakeup(ia_tid);
}