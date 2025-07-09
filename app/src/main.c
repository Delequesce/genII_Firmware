#include <stdio.h>
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
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
#include <app/circularbuffer.h>
#include <zephyr/dt-bindings/gpio/stm32-gpio.h>
#include <zephyr/sys/poweroff.h>

/* Default States */
enum testStates activeState = IDLE;
enum heaterStates heaterState = NOT_HEATING; 

static struct test_config test_cfg = {
	.runTime = DEFAULT_RUN_TIME,
	.collectionInterval = DEFAULT_COLLECTION_INTERVAL,
	.incubationTemp = DEFAULT_INCUBATION_TEMP,
	.channelOn = {1, 1, 1, 1}, // Change to set default active channels
	.boardNumber = 0,
};

/* Load with default values */
static struct calibration_data calibMat[4] = {
	{DEFAULT_ZFB_REAL, DEFAULT_ZFB_IMAG},
	{DEFAULT_ZFB_REAL, DEFAULT_ZFB_IMAG},
	{DEFAULT_ZFB_REAL, DEFAULT_ZFB_IMAG},
	{DEFAULT_ZFB_REAL, DEFAULT_ZFB_IMAG},
};

/* Channel to TIA Select mappings */
static const uint32_t tia_shdn_states[7] = {
	GPIO_OUTPUT_INACTIVE, GPIO_OUTPUT_INACTIVE,
	GPIO_OUTPUT_INACTIVE, GPIO_OUTPUT_ACTIVE,
	GPIO_OUTPUT_INACTIVE, GPIO_OUTPUT_INACTIVE,
	GPIO_OUTPUT_INACTIVE,
};

/* Main data structure */
static struct impedance_data testDataMat[DEFAULT_EQC_TIME][4] = {{0}};
static float C_Mat[N_AVERAGES] = {0};
static float G_Mat[N_AVERAGES] = {0};

static const struct impedance_data qcData[5][4] = {
	{{.C = 46.5, .G = 5.57}, {.C = 101.5, .G = 2.57}, {.C = 230.0, .G = 10.05}, {.C = 324.6, .G = 5.02}},
	{{.C = 48.0, .G = 6.688}, {.C = 229.9, .G = 10.080}, {.C = 98.8, .G = 2.570}, {.C = 326.6, .G = 6.686}},
	{{.C = 47.2, .G = 6.689}, {.C = 99.4, .G = 2.571}, {.C = 230.0, .G = 10.037}, {.C = 319.5, .G = 6.691}},
	{{.C = 47.2, .G = 6.692}, {.C = 100.1, .G = 2.572}, {.C = 224.9, .G = 10.033}, {.C = 324.3, .G = 6.683}},
	{{.C = 46.7, .G = 6.689}, {.C = 227.6, .G = 10.040}, {.C = 100.6, .G = 2.572}, {.C = 322.6, .G = 6.691}},
};

/* Threads */
k_tid_t ia_tid; // IA Measurement Thread
k_tid_t heater_tid;
k_tid_t uartio_tid;
struct k_thread IA_thread_data;
struct k_thread heater_thread_data;
struct k_thread uartio_thread_data;
K_THREAD_STACK_DEFINE(heater_stack_area, HEATER_STACK_SIZE);
//K_THREAD_DEFINE(heater_tid, HEATER_STACK_SIZE, heaterThread_entry_point, NULL, NULL, NULL, HEATER_THREAD_PRIORITY, 0, 0);
K_THREAD_STACK_DEFINE(uartio_stack_area, UARTIO_STACK_SIZE);
//K_THREAD_DEFINE(uartIO_tid, UARTIO_STACK_SIZE, uartIOThread_entry_point, NULL, NULL, NULL, UARTIO_THREAD_PRIORITY, 0, 0);
K_THREAD_STACK_DEFINE(IA_stack_area, IA_STACK_SIZE); 

/* Message and work queue */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 4, 1); // Message queue can handle 10 items of size MSG_SIZE (bytes), aligned to 1 byte boundary. 

/* Relevant Device Tree Structures */
const struct device *uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
static const struct device *flash_device = DEVICE_DT_GET(DT_CHOSEN(zephyr_flash_controller));
static const struct pwm_dt_spec ccDriver = PWM_DT_SPEC_GET(CCDRIVER);
static const struct device* ad4002_master = DEVICE_DT_GET(AD4002_INSTANCE_1);
static const struct device* ad4002_slave = DEVICE_DT_GET(AD4002_INSTANCE_2);
static const struct gpio_dt_spec adc_shdn_low = GPIO_DT_SPEC_GET(ADC_SHDN_LOW, gpios);
static const struct gpio_dt_spec tia_1_shdn_low = GPIO_DT_SPEC_GET(TIA1_SHDN_LOW, gpios);
static const struct gpio_dt_spec tia_2_shdn_low = GPIO_DT_SPEC_GET(TIA2_SHDN_LOW, gpios);
static const struct gpio_dt_spec tia_3_shdn_low = GPIO_DT_SPEC_GET(TIA3_SHDN_LOW, gpios);
static const struct gpio_dt_spec tia_4_shdn_low = GPIO_DT_SPEC_GET(TIA4_SHDN_LOW, gpios);
static const struct gpio_dt_spec charge_enable_high = GPIO_DT_SPEC_GET(CHARGE_ENABLE_HIGH, gpios);
static const struct gpio_dt_spec power_enable_low = GPIO_DT_SPEC_GET(POWER_ENABLE_LOW, gpios);

// Heater Options
static const struct pwm_dt_spec heaterPwm = PWM_DT_SPEC_GET(HEATERPWM);

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
		printk("EUart not ready\n");
		return -1;
	}

	/* Configure Device Params */
	ret = configure_uart_device(uart_dev);
	if (ret < 0){
		printk("EUart failed to initialize\n");
		return -1;
	}

	/* Turn off heater power and turn all amps and references on */
    if (!gpio_is_ready_dt(&adc_shdn_low)) {
        printk("ESHDN pins not ready");
		return 0;
	}

	if (gpio_pin_configure_dt(&adc_shdn_low, GPIO_OUTPUT_ACTIVE) < 0) {
        printk("ESHDN pins not properly configured");
		return 0;
	}

	/* Configure TIA SHDNs, setting all SHDN to start */
    if (!gpio_is_ready_dt(&tia_1_shdn_low) ||
		!gpio_is_ready_dt(&tia_2_shdn_low) ||
		!gpio_is_ready_dt(&tia_3_shdn_low) || 
		!gpio_is_ready_dt(&tia_4_shdn_low)) {
        printk("ETIA SHDN Pins not ready");
		return 0;
	}

	if (gpio_pin_configure_dt(&tia_1_shdn_low, GPIO_OUTPUT_INACTIVE) < 0 || 
				gpio_pin_configure_dt(&tia_2_shdn_low, GPIO_OUTPUT_INACTIVE) < 0 || 
				gpio_pin_configure_dt(&tia_3_shdn_low, GPIO_OUTPUT_INACTIVE) < 0 ||
				gpio_pin_configure_dt(&tia_4_shdn_low, GPIO_OUTPUT_INACTIVE) < 0) {
				printk("ETIA Multiplexing Error");
				return 0;
		}

	/* Set up power to Raspberry Pi (Power enable) and battery charging (charge enable) GPIOs */
	if (!gpio_is_ready_dt(&charge_enable_high) || gpio_pin_configure_dt(&charge_enable_high, GPIO_OUTPUT_ACTIVE) < 0){
		printk("ECannot set charge enable, Battery Charging disabled");
	}

	if (!gpio_is_ready_dt(&power_enable_low) || gpio_pin_configure_dt(&power_enable_low, GPIO_OUTPUT_INACTIVE) < 0){
		printk("ECannot enable power input to Pi.");
		return 0;
	}

	// Create Heater and Uart Threads
	heater_tid = k_thread_create(&heater_thread_data, heater_stack_area,
		K_THREAD_STACK_SIZEOF(heater_stack_area),
		heaterThread_entry_point, 
		NULL, NULL, NULL, 
		HEATER_THREAD_PRIORITY, 0, K_NO_WAIT);

	uartio_tid = k_thread_create(&uartio_thread_data, uartio_stack_area,
		K_THREAD_STACK_SIZEOF(uartio_stack_area),
		uartIOThread_entry_point, 
		NULL, NULL, NULL, 
		UARTIO_THREAD_PRIORITY, 0, K_NO_WAIT);

	return 0; // Scheduler invokes highest priority ready thread, which is uartIOThread (goes to entry point)
}

int compare(const void* a, const void* b){
	float fa = *(const float*) a;
	float fb = *(const float*) b;
	return COMPARE(fa, fb);
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
			printk("EInterrupt-driven UART API support not enabled\n");
		} else if (ret == -ENOSYS) {
			printk("EUART device does not support interrupt-driven API\n");
		} else {
			printk("EError setting UART callback: %d\n", ret);
		}
		return 0;
	}

	uart_irq_rx_enable(uart_dev);
	
	return 0;


}

/* Polling based UART handler thread */
static void uartIOThread_entry_point(){
	//printk("EUart Thread Starting\n");

	while(1){
		/* Check messagequeue */
		while (k_msgq_num_used_get(&uart_msgq)) {
			unsigned char p_char[MSG_SIZE]; 
        	k_msgq_get(&uart_msgq, &p_char, K_NO_WAIT);

			/* Decides how to process data based on control character */
			switch(p_char[0]) {
				if (activeState == IDLE){
					case 'C': // Connect to Device
						uart_write_singleChar('K', true);
						//deviceConnected = false;
						deviceConnected = true;
						break;
					case 'S': // Alter Test Configuration Structure, convert from ascii encoding
						test_cfg.runTime = 1000 * p_char[1] + 100 * p_char[2] + 10 * p_char[3] + p_char[4] - 53328;
						test_cfg.collectionInterval = p_char[5]-48;
						test_cfg.incubationTemp = 10 * p_char[6] + p_char[7] - 528;
						//test_cfg.channels
						// Respond positively
						uart_write_singleChar('K', true);
						break;
					case 'L':
						test_cfg.channelOn[0] = p_char[1] & 0x1;
						test_cfg.channelOn[1] = p_char[1] & 0x2;
						test_cfg.channelOn[2] = p_char[1] & 0x4;
						test_cfg.channelOn[3] = p_char[1] & 0x8;
						break;
					case 'N': // New Test
						activeState = TESTRUNNING;
						/* Allocate memory and spawn new thread */
						ia_tid = k_thread_create(&IA_thread_data, IA_stack_area,
										K_THREAD_STACK_SIZEOF(IA_stack_area),
										testThread_entry_point, 
										&test_cfg, NULL, NULL, 
										IA_THREAD_PRIORITY, 0, K_NO_WAIT);

						// Respond Positively
						uart_write_singleChar('K', true);
						break;
					case 'H': // Toggle Heater
						/* Toggle global parameter for heater thread to observe */
						heaterState = p_char[1] - 48;
						//heaterState = (heaterState + 1) % 2;
						heater_errI = 0; // Reset Integral counter
						if (heaterState == NOT_HEATING){
							//pwm_set_cycles(heaterPwm.dev, heaterPwm.channel, V_SIG_PERIOD, V_SIG_PERIOD, heaterPwm.flags);
							pwm_set_cycles(heaterPwm.dev, heaterPwm.channel, V_SIG_PERIOD, V_SIG_PERIOD, heaterPwm.flags);
						}
						// Acknowledge Request
						uart_write_singleChar('K', true);
						break;
					case 'B': // Calibrate System
						activeState = CALIBRATING;
						struct test_config calib_cfg = {
							.runTime = DEFAULT_CALIBRATION_TIME,
							.collectionInterval = DEFAULT_COLLECTION_INTERVAL,
							.incubationTemp = 0,
							.channelOn = {1, 1, 1, 1},
							.boardNumber = p_char[1] - 48,
						};

						// Respond positively
						uart_write_singleChar('K', true);

						ia_tid = k_thread_create(&IA_thread_data, IA_stack_area,
										K_THREAD_STACK_SIZEOF(IA_stack_area),
										testThread_entry_point, 
										&calib_cfg, NULL, NULL, 
										IA_THREAD_PRIORITY, 0, K_NO_WAIT);
						break;
					case 'Q': // Run EQC
						activeState = EQC;
						struct test_config eqc_cfg = {
							.runTime = DEFAULT_EQC_TIME,
							.collectionInterval = DEFAULT_COLLECTION_INTERVAL,
							.incubationTemp = 0,
							.channelOn = {1, 1, 1, 1},
							.boardNumber = p_char[1] - 48,
						};

						// Respond positively
						uart_write_singleChar('K', true);

						ia_tid = k_thread_create(&IA_thread_data, IA_stack_area,
										K_THREAD_STACK_SIZEOF(IA_stack_area),
										testThread_entry_point, 
										&eqc_cfg, NULL, NULL, 
										IA_THREAD_PRIORITY, 0, K_NO_WAIT);
						break;
					case 'Y': // Disconnect
						uart_write_singleChar('K', true);
						deviceConnected = false;
						break;
					default:
						// Respond negatively
						uart_write_singleChar('V', true);
				}
				else{
					case 'X': // Stop Test
						if (stopTest() < 0){
							break;
						}
						// Acknowledge
						uart_write_singleChar('K', true);
						break;
				}
			}
		}
		/* Yield to newly spawned thread or to heater thread */
		k_msleep(500);
	}
	return;
}

static void heaterThread_entry_point(void *unused1, void *unused2, void *unused3){

	/* Casts unused params to void to avoid compiler warnings */
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	ARG_UNUSED(unused3);

	/* Ensure Channel is held HIGH until heating begins */
	pwm_set_cycles(heaterPwm.dev, heaterPwm.channel, V_SIG_PERIOD, V_SIG_PERIOD, heaterPwm.flags);

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
	for (size_t i = 0U; i < NUM_THERMISTOR_CHANNELS; i++) {
		if (!adc_is_ready_dt(&adc_channels[i])) {
			printk("EADC controller device %s not ready\n", adc_channels[i].dev->name);
			return -1;
		}

		if (adc_channel_setup_dt(&adc_channels[i]) < 0) {
			printk("ECould not setup channel #%d\n", i);
			return -1;
		}
	}
	float heater_errP, heater_errD;
	heater_errI = 0;
	int err;
	float tempAvg = 0;
	float CCTemp = 0;
	float prevTempAvg = 0;
	float prevCCTemp = 0;
	bool heater_on_off = false; 
	volatile int64_t sleepTime, timeStamp; // Timing params for measuring speed

	//static bool heatFlag = 0;

	/* Initial Read */
	tempAvg = readTemp(&sequence);
	CCTemp = tempAvg;
	prevTempAvg = tempAvg;
	prevCCTemp = tempAvg;
	uint8_t tempWriteFlag = 0;

	int64_t startTime = k_uptime_get();

	while(1){
		
		/* This operation takes around 400 msec */
		tempAvg = readTemp(&sequence);

		/* Send temperature reading to GUI */
		if(deviceConnected){
			if (tempWriteFlag == 1){
				uart_write_32f(&tempAvg, 1, 'T');
				tempWriteFlag = 0;
			}
			tempWriteFlag++;
		}

		if(heaterState == HEATING){
			
			/* PID */
 			heater_errP = test_cfg.incubationTemp-tempAvg;
			if (heater_errP > FULL_POWER_ERR_THRESH){
				heater_errI = 0;
				pulse_cycles = 0;
			}
			else{
				/* Find PID errors and calculate output duty cycle */
				heater_errI = heater_errI + heater_errP;
				heater_errD = prevTempAvg - tempAvg;
				prevTempAvg = tempAvg;

				#if CHIP_HEATER
				pulse_cycles = (uint32_t)(V_SIG_PERIOD * (1-(K_P * heater_errP + K_I * heater_errI + K_D * heater_errD)*0.01));
				#else
				pulse_cycles = (uint32_t)(V_SIG_PERIOD * (1- 0.01*K_C*(heater_errP + K_I * heater_errI)));
				#endif
				pulse_cycles = pulse_cycles > V_SIG_PERIOD ? V_SIG_PERIOD:pulse_cycles;
				pulse_cycles = pulse_cycles < 0 ? 0:pulse_cycles;
			}
			//printk("%0.2f,%d,%0.2f,%0.2f\n", tempAvg, pulse_cycles, K_C*heater_errP, K_C * K_I* heater_errI);
			if (pwm_set_cycles(heaterPwm.dev, heaterPwm.channel, V_SIG_PERIOD, pulse_cycles, heaterPwm.flags) < 0){
				printk("EError: Failed to set heater pulse");
				return -1;
			}

		}

		/* Schedule new reading every second and let other threads run */
		timeStamp = k_uptime_get() - startTime;
		//printk("Timestamp: %lld", timeStamp);
		sleepTime = TEMP_COLLECTION_INTERVAL * 1000*(count+1) - timeStamp;
		count++;
		k_msleep(sleepTime);
	}
	return;
}

/* Performs measurements */
static void testThread_entry_point(const struct test_config* test_cfg, void *unused1, void *unused2){

	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);

	/* Turn turn all amps and references on */
    if (!gpio_is_ready_dt(&adc_shdn_low)) {
        printk("ESHDN pins not ready");
		return 0;
	}
    if (gpio_pin_configure_dt(&adc_shdn_low, GPIO_OUTPUT_ACTIVE) < 0) {
        printk("ESHDN pins not properly configured");
		return 0;
	}

	/** Get Current Calibration Data from Flash and read into calibration structure 
	 * If normal test is running, load the most recent calibration. 
	 * If test is for calibration, use default values for easy calibration. 
	*/
	if (activeState != CALIBRATING){
		flash_read(flash_device, PAGE200, &calibMat, 32);
	}

	/* Data Buffer Initialization */
	static uint16_t Ve_data[SAMPLES_PER_COLLECTION]; // Memory allocation for ADC RX Data  
    static uint16_t Vr_data[SAMPLES_PER_COLLECTION]; // Memory allocation for ADC RX Data 
	static uint16_t Ve_data_safe[SAMPLES_PER_COLLECTION]; // Memory allocation for ADC RX Data  
    static uint16_t Vr_data_safe[SAMPLES_PER_COLLECTION]; // Memory allocation for ADC RX Data 

	static struct impedance_data testDataMat_lite[4] = {0};

	static struct statValues sVals[4] = {0};

	/* Enable ADC and CC Drive */
	if (pwm_set_cycles(ccDriver.dev, ccDriver.channel, V_SIG_PERIOD, V_SIG_PERIOD/2, ccDriver.flags) < 0){
		printk("EError: Failed to setup CC Drive");
		return -1;
	}

	/* Begin Measurement */
	uint16_t N_Measurements;
	if(test_cfg){
		N_Measurements = (uint16_t)(test_cfg->runTime / test_cfg->collectionInterval);
	}
	uint16_t i, j, c;
	uint16_t n;
	const float w0 = W0;//2*PI*(1-DEFAULT_SPOT_FREQUENCY/CONVERT_FREQUENCY); 
	const float cos_w0 = cosf(w0);
	const float sin_w0 = sinf(w0);
	float mag2Vr, mag2Ve, mag2Z, Z_temp_real, Z_temp_imag;
	float Vr_real, Vr_imag, Ve_real, Ve_imag;
	float Z_real, Z_imag = 0;
	float Z_real_mean[4] = {0, 0, 0, 0};
	float Z_imag_mean[4] = {0, 0, 0, 0};
	float prev_value_Vr, prev_value_Ve;
	float prev_prev_value_Vr, prev_prev_value_Ve;
	float current_value_Vr, current_value_Ve;

	struct dataWriteStruct dwStruct[N_CHANNELS_MAX] = {{
		.timeStamp = {0},
		.impDat = {0}, 
	}};

	unsigned char a_char;


	/* Configure TIA SHDNs */
    if (!gpio_is_ready_dt(&tia_1_shdn_low) ||
		!gpio_is_ready_dt(&tia_2_shdn_low) ||
		!gpio_is_ready_dt(&tia_3_shdn_low) || 
		!gpio_is_ready_dt(&tia_4_shdn_low)) {
        printk("ETIA SHDN Pins not ready");
		return 0;
	}

	/* Sets up necessary peripherals (DMA, SPI, Timers) for reads. */
	ad4002_init_read(ad4002_master, ad4002_slave, Ve_data, Vr_data, SAMPLES_PER_COLLECTION);
	ad4002_irq_callback_set(ad4002_master, &dma_tcie_callback);
	volatile int64_t sleepTime, timeStamp; // Timing params for measuring speed

	/* Timing Parameters */
	int64_t startTime = k_uptime_get();

	/* This loop runs each collection for the entire test run time (outer loop) */
	for(i = 0; i < N_Measurements; i++){

		/* This loop runs to obtain repeat measurements over the collection frequency interval */

		/* Run Loop for Each Channel */
		for(c = 0; c < N_CHANNELS_MAX; c++){
			
			/* If channel is not active, skip collection */
			if(!test_cfg->channelOn[c]){
				continue;
			}

			/* Set Active Channel by setting shdn pins */
			if (gpio_pin_configure_dt(&tia_1_shdn_low, tia_shdn_states[c+3]) < 0 || 
				gpio_pin_configure_dt(&tia_2_shdn_low, tia_shdn_states[c+2]) < 0 || 
				gpio_pin_configure_dt(&tia_3_shdn_low, tia_shdn_states[c+1]) < 0 ||
				gpio_pin_configure_dt(&tia_4_shdn_low, tia_shdn_states[c]) < 0) {
				printk("ETIA Multiplexing Error");
				return 0;
			}

			/* Perform Initial Read */
			ad4002_start_read(ad4002_master, SAMPLES_PER_COLLECTION);
			k_msleep(2); // Thread sleeps until DMA callback is triggered

			for(j = 0; j < N_AVERAGES; j++){

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

				/* Run Goertzl algorithm to get 1 MHz FFT bin */
				//t3 = k_uptime_get();
				for(n = SAMPLES_PER_COLLECTION-N_FFT; n < SAMPLES_PER_COLLECTION; n++){
					current_value_Ve = (Ve_data_safe[n] & RESOLUTION_MASK) + 2 * cos_w0 * prev_value_Ve - prev_prev_value_Ve;
					prev_prev_value_Ve = prev_value_Ve;
					prev_value_Ve = current_value_Ve; 

					current_value_Vr = (Vr_data_safe[n] & RESOLUTION_MASK) + 2 * cos_w0 * prev_value_Vr - prev_prev_value_Vr;
					prev_prev_value_Vr = prev_value_Vr;
					prev_value_Vr = current_value_Vr; 
				}
				//t4 = k_uptime_get();
				/* Final Step calcul$3 = {24577, 44022, 29183, 39204, 38089, 36787, 35435, 33892, 32227, 30513, 28771, 27027, 25297, 23753, 22482, 21574, 21225, 21311, 21963, 22860, 24030, 25264, 26646, 28187, 29762, 31522, 33235, 35020, 36760, 38297, 39566, 40472, 40879, 40773, 40152, 39202, 38075, 36808, 35424, 33902, 32278, 30547, 28763, 26986, 25305, 23704, 22479, 21616, 21194, 21324, 21895, 22876, 24012, 25260, 26631, 28169, 29794, 31493, 33272, 35049, 36738, 38278, 39551, 40460, 40874, 40767, 40164, 39209, 38059, 36812, 35414, 33905, 32266, 30551, 28734, 26985, 25275, 23754, 22470, 21580, 21182, 21321, 21918, 22859, 24013, 25255, 26636, 28141, 29787, 31475, 33289, 35035, 36753, 38292, 39574, 40449, 40910, 40783, 40140, 39224, 38068, 36798, 35423, 33892, 32254, 30499, 28769, 26982, 25307, 23722, 22478, 21616, 21195, 21322, 21926, 22869, 24004, 25246, 26646, 28166, 29804, 31477, 33276, 35047, 36752, 38277, 39568, 40437, 40881, 40774, 40157, 39205, 38082, 36808, 35398, 33933, 32256, 30544, 28761, 27001, 25307, 23702, 22510, 21612, 21201, 21306, 21911, 22869, 23994, 25258, 26643, 28181, 29772, 31478, 33244, 35028, 36753, 38295, 39582, 40471, 40874, 40769, 40160, 39185, 38073, 36791, 35427, 33904, 32267, 30552, 28778, 26982, 25305, 23710, 22481, 21575, 21209, 21304, 21932, 22884, 24007, 25246, 26643, 28199, 29796, 31487, 33238, 35041, 36732, 38311, 39548, 40464, 40893, 40776, 40165, 39212, 38083, 36802, 35417, 33931, 32271, 30545, 28752, 26990, 25305, 23749, 22466, 21596, 21209, 21325, 21928, 22881, 24013, 25266, 26612, 28156, 29778, 31510, 33283, 35043, 36740, 38306, 39552, 40471, 40911, 40762, 40170, 39199, 38076, 36780, 35435, 33898, 32268, 30525, 28751, 26982, 25311, 23730, 22510, 21596, 21203, 21319, 21925, 22867, 24015, 25245, 26657, 28171, 29804, 31499, 33258, 35073, 36751, 38287, 39574, 40471, 40888, 40784, 40171, 39218, 38070, 36798, 35454, 33935, 32271, 30520, 28765, 26987, 25284, 23716, 22464, 21603, 21180, 21321, 21952, 22880, 24034, 25274, 26641, 28175, 29794, 31514, 33265, 35046, 36742, 38286, 39573, 40450, 40882, 40770, 40164, 39212, 38067, 36800, 35413, 33899, 32270, 30542, 28764, 26992, 25276, 23757, 22473, 21596, 21181, 21317, 21944, 22888, 24019, 25257, 26626, 28179, 29795, 31485, 33256, 35021, 36746, 38310, 39571, 40468, 40890, 40768, 40166, 39224, 38078, 36831, 35419, 33912, 32248, 30527, 28745, 26983, 25315, 23730, 22472, 21611, 21196, 21299, 21944, 22858, 24008, 25248, 26626, 28162, 29816, 31483, 33265, 35034, 36757, 38291, 39578, 40469, 40878, 40765, 40166, 39190, 38078, 36808, 35421, 33892, 32274, 30506, 28770, 26996, 25277, 23768, 22484, 21580, 21191, 21316, 21941, 22877, 24006, 25270, 26631, 28176, 29785, 31489, 33263, 35057, 36720, 38291, 39568, 40450, 40876, 40771, 40161, 39220, 38076, 36794, 35418, 33918, 32257, 30529, 28761, 26998, 25307, 23724, 22442, 21608, 21185, 21322, 21931, 22881, 24011, 25259, 26633, 28170, 29804, 31506, 33276, 35045, 36745, 38297, 39577, 40450, 40863, 40762, 40162, 39221, 38077, 36815, 35430, 33894, 32281, 30541, 28764, 26995, 25284, 23725, 22465, 21595, 21190, 21349, 21924, 22897, 23985, 25259, 26639, 28170, 29777, 31470, 33254, 35029, 36745, 38276, 39567, 40480, 40884, 40767, 40179, 39236, 38075, 36817, 35421, 33912, 32256, 30538, 28757, 26983, 25276, 23759, 22484, 21567, 21189, 21317, 21932, 22868, 24004, 25235, 26649, 28164, 29781, 31498, 33289, 35023, 36742, 38290, 39569, 40459, 40871, 40769, 40167, 39239, 38062, 36799, 35453, 33909, 32285, 30540, 28769, 26984, 25287, 23739, 22475, 21606, 21195, 21323, 21936, 22862, 24008, 25246, 26637, 28175, 29815, 31489, 33283, 35051, 36757, 38290, 39573, 40452, 40887, 40750, 40169, 39219, 38093, 36806, 35427, 33903, 32259, 30534, 28762, 26999, 25311, 23711, 22469, 21581, 21199, 21312, 21912, 22874, 24002, 25247, 26625, 28179, 29788, 31473, 33234, 35059, 36758, 38302, 39559, 40457, 40879, 40770, 40181, 39211, 38090, 36801, 35420, 33910, 32254, 30511, 28758, 27002, 25309, 23712, 22447, 21596, 21223, 21306, 21929, 22852, 24002, 25257, 26661, 28187, 29800, 31501, 33270, 35045, 36751, 38302, 39568, 40447, 40866, 40771, 40174, 39213, 38075, 36808, 35441, 33918, 32255, 30525, 28763, 27010, 25268, 23769, 22489, 21589, 21211, 21319, 21929, 22863, 23992, 25246, 26636, 28156, 29803, 31479, 33246, 35048, 36735, 38288, 39562, 40469, 40874, 40774, 40166, 39220, 38044, 36806, 35426, 33891, 32254, 30525, 28785, 26988, 25283, 23713, 22423, 21578, 21189, 21321, 21926, 22888, 23993, 25250, 26658, 28163, 29781, 31520, 33282, 35065, 36748, 38293, 39566, 40454, 40866, 40759, 40156, 39224, 38077, 36815, 35428, 33891, 32266, 30551, 28763, 27001, 25313, 23745, 22470, 21602, 21198, 21311, 21939, 22875, 23978, 25249, 26640, 28150, 29791, 31489, 33261, 35047, 36742, 38275, 39573, 40443, 40863, 40754, 40172, 39221, 38075, 36820, 35421, 33882, 32259, 30533, 28762, 26978, 25311, 23731, 22476, 21609, 21174, 21316, 21953, 22850, 24018, 25274, 26659, 28168, 29767, 31479, 33265, 35051, 36751, 38286, 39586, 40480, 40869, 40773, 40150, 39214, 38081, 36821, 35394, 33905, 32247, 30523, 28759, 27002, 25306, 23721, 22479, 21601, 21204, 21313, 21958, 22895, 24005, 25248, 26656, 28178, 29784, 31522, 33255, 35027, 36757, 38316, 39558, 40466, 40881, 40758, 40165, 39207, 38069, 36814, 35420, 33882, 32258, 30527, 28751, 26979, 25316, 23718, 22467, 21580, 21186, 21333, 21931, 22860, 24016, 25237, 26643, 28171, 29769, 31475, 33258, 35025, 36744, 38278, 39591, 40469, 40889, 40789, 40168, 39209, 38064, 36793, 35417, 33916, 32258, 30548, 28756, 26990, 25328, 23744, 22471, 21609, 21193, 21322, 21944, 22874, 24022, 25256, 26651, 28188, 29788, 31498, 33255, 35029, 36738, 38305, 39548, 40471, 40873, 40749, 40174, 39224, 38062, 36791, 35394, 33894, 32260, 30544, 28743, 26978, 25280, 23719, 22468, 21580, 21192, 21323, 21922, 22891, 24002, 25237, 26644, 28179, 29789, 31479, 33277, 35066, 36740, 38274, 39554, 40434, 40875, 40786, 40148, 39200, 38075, 36792, 35444, 33894, 32238, 30524, 28763, 26980, 25312, 23733, 22480, 21577, 21175, 21334, 21927, 22885, 24001, 25275, 26669, 28187, 29756, 31492, 33254, 35021, 36751, 38292, 39572, 40458, 40873, 40764, 40170, 39225, 38071, 36809, 35418, 33916, 32246, 30519, 28789, 26990, 25320, 23745, 22469, 21583, 21202, 21329, 21907, 22878, 24012, 25244, 26641, 28184, 29789, 31487, 33245, 35042, 36742, 38294, 39576, 40451, 40896, 40758, 40175, 39220, 38069, 36803, 35421, 33924, 32267, 30539, 28764, 27001, 25298, 23740, 22484, 21579, 21178, 21328, 21921, 22875, 24017, 25267, 26638, 28154, 29797, 31494, 33287, 35050, 36717, 38275, 39570, 40464, 40883, 40757, 40143, 39195, 38063, 36822, 35416, 33912, 32261, 30544, 28782, 26982, 25294, 23727, 22469, 21590, 21209, 21309, 21938, 22881, 23996, 25267, 26675, 28131, 29790, 31498, 33263, 35041, 36725, 38299, 39588, 40447, 40872, 40778, 40148, 39229, 38078, 36817, 35419, 33923, 32274, 30542, 28758, 26965, 25309, 23725, 22451, 21594, 21153, 21342, 21924, 22894, 24002, 25262, 26639, 28170, 29797, 31488, 33276, 35034, 36740, 38291, 39583, 40471, 40884, 40745, 40156, 39225, 38067, 36807, 35425, 33909, 32264, 30556, 28736, 26994, 25311, 23705, 22495, 21589, 21191, 21304, 21940, 22865, 23998, 25268, 26648, 28146, 29800, 31473, 33283, 35029, 36764, 38304, 39555, 40463, 40883, 40761, 40182, 39209, 38077, 36801, 35422, 33919, 32252, 30514, 28741, 26989, 25305, 23724, 22463, 21589, 21195, 21327, 21918, 22882, 23983, 25256, 26635, 28167, 29761, 31481}
ates complex FFT bin for each signal */
				Ve_real = 3*(cos_w0 * prev_value_Ve - prev_prev_value_Ve)/N_FFT/65535.0f;
				Ve_imag = 3*(sin_w0 * prev_value_Ve)/N_FFT/65535.0f;

				Vr_real = 3*(cos_w0 * prev_value_Vr - prev_prev_value_Vr)/N_FFT/65535.0f;
				Vr_imag = 3*(sin_w0 * prev_value_Vr)/N_FFT/65535.0f;

				/* Finally, compute Z */
 				Z_temp_real = COMPLEX_DIVIDE_REAL(Ve_real, Ve_imag, Vr_real, Vr_imag);
				Z_temp_imag = COMPLEX_DIVIDE_IMAG(Ve_real, Ve_imag, Vr_real, Vr_imag);
				
				// Calculate Z with calibration values and offsets
				Z_real = COMPLEX_MULTIPLY_REAL(Z_temp_real, Z_temp_imag, calibMat[c].Zfb_real, calibMat[c].Zfb_imag) + Z_OFF_REAL;
				Z_imag = COMPLEX_MULTIPLY_IMAG(Z_temp_real, Z_temp_imag, calibMat[c].Zfb_real, calibMat[c].Zfb_imag) + Z_OFF_IMAG;

				// Calculate C and G and store in matrix
				mag2Z = Z_real*Z_real + Z_imag*Z_imag;
				G_Mat[j] = 1000 * Z_real/mag2Z;				// Result is in mS
				C_Mat[j] = 159154.943091895f * Z_imag/mag2Z; // magic number is 1e12 / (2*pi*1e6). Result is in pF
			}
			
			// Calculate Statistics for data set
			dwStruct[c].timeStamp = (float)(k_uptime_get() - startTime);
			qsort(C_Mat, N_AVERAGES, sizeof(float), compare);
			qsort(G_Mat, N_AVERAGES, sizeof(float), compare);
			testDataMat_lite[c].G = G_Mat[N_AVERAGES/2]; // Overwritten each timepoint
			testDataMat_lite[c].C = C_Mat[N_AVERAGES/2];

			// sVals[c].median = C_Mat[N_AVERAGES/2];
			// sVals[c].min = C_Mat[0];
			// sVals[c].max = C_Mat[N_AVERAGES-1];
			// for(j = 0; j < N_AVERAGES; j++){
			// 	sVals[c].mean = (sVals[c].mean*j + C_Mat[j])/(j+1);
			// }
			// for(j = 0; j < N_AVERAGES; j++){
			// 	sVals[c].var += pow(C_Mat[j] - sVals[c].mean, 2);
			// }
			// sVals[c].var = sVals[c].var/N_AVERAGES;

			/* Package into data structures */
			dwStruct[c].impDat = testDataMat_lite[c];
			testDataMat[i][c].C = testDataMat_lite[c].C;
			testDataMat[i][c].G = testDataMat_lite[c].G;
		}

		/* Send each time point's data over uart */
		if (activeState == TESTRUNNING){
			/* Total write at 115200 baud should take 8-10 msec */
			uart_write_32f(&dwStruct[0], 12, 'D');
			// uart_write_32f(&sVals, 20, 'D');
		}

		/* Collection timestamp */
		timeStamp = k_uptime_get() - startTime;

		/* Sleep until next collection period */
		sleepTime = (test_cfg->collectionInterval) * 1000*(i+1) - timeStamp;
		
		k_msleep(sleepTime); // Usually around 270 msec
	}

	/* Once done with measurements, take care of data for Calibration and EQC */
	if(activeState == TESTRUNNING){
		/* Tell UI that test has completed */
		activeState = IDLE;
		uart_write_singleChar('X', true);
		return; 
	}

	/* Back-calculate Z Real and Z Imag from C and G */
	float Y_real;
	float Y_imag; 
	float mag2Y;
	for(c = 0; c < N_CHANNELS_MAX; c++){

		for(i = 0; i < DEFAULT_CALIBRATION_TIME; i++){
			Y_real = 0.001f * testDataMat[i][c].G;
			Y_imag = 0.0000062832f * testDataMat[i][c].C;
			mag2Y = Y_real*Y_real + Y_imag*Y_imag;
			Z_real_mean[c] = (Z_real_mean[c] * i + Y_real/mag2Y)/(i+1);
			Z_imag_mean[c] = (Z_imag_mean[c] * i - Y_imag/mag2Y)/(i+1);
		}
	}

	/* Perform additional calibration steps, if necessary */
	if(activeState == CALIBRATING){
		// Rev 2
		//const float test_Z_real = 179.636; 
		//const float test_Z_imag = 0.043; 
		// Rev 3
		const float test_Z_real = 149.477; 
		const float test_Z_imag = 0.070;

		const float expected_calib_real = -63;
		const float expected_calib_imag = 20;

		float div_temp_real;
		float div_temp_imag;

		/* Read current calibration values as reference */
		flash_read(flash_device, PAGE200, &calibMat, 32);

		for(c=0; c<4; c++){
			if (test_cfg->channelOn[c]){
				div_temp_real = COMPLEX_DIVIDE_REAL(test_Z_real, test_Z_imag, Z_real_mean[c], Z_imag_mean[c]);
				div_temp_imag = COMPLEX_DIVIDE_IMAG(test_Z_real, test_Z_imag, Z_real_mean[c], Z_imag_mean[c]);

				/* Store Values */
				calibMat[c].Zfb_real = div_temp_real;
				calibMat[c].Zfb_imag = div_temp_imag;
			}
		}

		/* Send new values over uart*/
		uart_write_32f(&calibMat, 8, 'C');

		/* Write to flash memory */
		//flash_write_protection_set(flash_device, false);
		flash_erase(flash_device, PAGE200, 32);
		flash_write(flash_device, PAGE200, &calibMat, 32);
		printk("ECalibration Values Written to Memory\n");
		//flash_write_protection_set(flash_device, true);
		activeState = IDLE;
		return;
	}

	/* Compare qcData struct to average of testDataMat */
	if(activeState == EQC){
		uint8_t qcIndex = test_cfg->boardNumber; // Get Board Number Entered by user from GUI
		float C_sum[4] = {0};
		float G_sum[4] = {0};
		float C_var[4] = {0};
		float G_var[4] = {0};
		struct impedance_data rmsd_noise[2] = {{.C = 0, .G = 0}};
		
		// Get Mean and RMS Deviation
		for(c = 0; c < 4; c++){
			for(i = 0; i < N_Measurements; i++){
				C_sum[c] += testDataMat[i][c].C;
				G_sum[c] += testDataMat[i][c].G;
			}
			C_sum[c] = C_sum[c] * 0.0333333f; 
			G_sum[c] = G_sum[c] * 0.0333333f;

			rmsd_noise[0].C += pow(qcData[qcIndex][c].C - C_sum[c], 2);
			rmsd_noise[0].G += pow(qcData[qcIndex][c].G - G_sum[c], 2);
		}
		// Get Standard Deviation (noise)
		for(c = 0; c < 4; c++){
			for(i = 0; i < N_Measurements; i++){
				C_var[c] += pow(testDataMat[i][c].C - C_sum[c], 2);
				G_var[c] += pow(testDataMat[i][c].G - G_sum[c], 2);
			}
			rmsd_noise[1].C += C_var[c] * 0.033f;
			rmsd_noise[1].G += G_var[c] * 0.033f;
		}
		// Calculate RMSD and noise across 4 chips
		rmsd_noise[0].C = 0.5 * sqrt(rmsd_noise[0].C) * 0.3; // Magic numbers are to normalize over range and convert to %
		rmsd_noise[0].G = 0.5 * sqrt(rmsd_noise[0].G) * 12.5;

		rmsd_noise[1].C = 0.5 * sqrt(rmsd_noise[1].C); // In pF
		rmsd_noise[1].G = 0.5 * sqrt(rmsd_noise[1].G);

		// Write to User
		uart_write_32f(&rmsd_noise, 4, 'Q');

		// Return
		activeState = IDLE;
		return;
	}
}

static void uart_write_singleChar(char character, bool useLF){
	uart_poll_out(uart_dev, character);
	if(useLF){
		uart_poll_out(uart_dev, '\n');
	}
	return;
}

/* General write function that takes in a pointer to a 32b data, the number of data, and an id code */
static void uart_write_32f(float* data, uint8_t numData, char messageCode){
	
	char buffer[9];

	uint8_t n, i, k = 0;

	uart_poll_out(uart_dev, messageCode);
	for (i = 0; i < numData; i++){
		n = snprintf(buffer, 9, "%f", *data);
		n = (n > 9) ? 9: n;
	
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
	if (activeState == TESTRUNNING){
		activeState = IDLE;
		/* Turn off CC Drive signal */
		if (pwm_set_cycles(ccDriver.dev, ccDriver.channel, V_SIG_PERIOD, 0, ccDriver.flags) < 0){
			printk("EFailed to Cancel Drive Signal");
			return -1;
		}
		/* Shut down all TIAs */
		if (gpio_pin_configure_dt(&tia_1_shdn_low, GPIO_OUTPUT_INACTIVE) < 0 || 
				gpio_pin_configure_dt(&tia_2_shdn_low, GPIO_OUTPUT_INACTIVE) < 0 || 
				gpio_pin_configure_dt(&tia_3_shdn_low, GPIO_OUTPUT_INACTIVE) < 0 ||
				gpio_pin_configure_dt(&tia_4_shdn_low, GPIO_OUTPUT_INACTIVE) < 0) {
				printk("EFailed to Shut down TIAs");
				return -1;
			}
		/* Shut down ADC Power */
		if (gpio_pin_set_dt(&adc_shdn_low, 0) < 0) {
        	printk("EADC Power not shut down");
			return -1;
		}
		/* Turn off ADCs */
		ad4002_shutdown(ad4002_master);

		k_thread_abort(ia_tid);
	}
	activeState = IDLE;
	return 0; 
}

/* Read current temperature values from ADC */
static float readTemp(struct adc_sequence* sequence){
	int err;
	int16_t* val_mv_ptr;
	int32_t val_mv;

	/* Chip Thermistors (R25 = 2000, B = 3250, R = 1000)*/
	#if CHIP_HEATER
	float m_temp = 0.0349;
	float b_temp = -13.4;
	#else
	/* Axial Thermistors (R25 = 10000, B = 3950, R = 8000)*/
	float m_temp = 0.0287;
	float b_temp = -17.8;
	#endif
	//int64_t start_time = k_uptime_get();
	//int64_t timeStamp = 0;
	float channel_temps_local[NUM_THERMISTORS];
	channel_temps_local[0] = 0;
	float tempAvg[NUM_TEMP_READS] = {0};
	/* Each read takes around 40 msec, so 10 total reads is 400 msec*/
	for (uint8_t j = 0; j < NUM_TEMP_READS; j++){
		for (size_t i = 0U; i < NUM_THERMISTORS; i++) {

			(void)adc_sequence_init_dt(&adc_channels[i], sequence);

			err = adc_read_dt(&adc_channels[i], sequence);
			if (err < 0) {
				printk("ECould not read (%d)\n", err);
				continue;
			}
			
			val_mv_ptr = sequence->buffer; // Can't dereference a generic pointer, so have to cast to int16_t
			val_mv = (int32_t)(*val_mv_ptr & 0xFFFF); // Cast the 16b data to 32b
			err = adc_raw_to_millivolts_dt(&adc_channels[i], &val_mv);
			if (err < 0) {
				printk("EValue in mV not available\n");
			}
			else{
				float tempValue = (val_mv) * m_temp + b_temp;
				channel_temps_local[i] = tempValue;
			}
			//timeStamp = k_uptime_get() - start_time;
			//printk("Read Time: %lld", timeStamp);
		}

		for (uint8_t i = 0; i < NUM_THERMISTORS; i++){
			tempAvg[j] = (channel_temps_local[i] + tempAvg[j] * i)/(i+1);
		}
	}

	// Mean
	// Median
	qsort(tempAvg, NUM_TEMP_READS, sizeof(float), compare);

	return (THERMISTOR_SCALING * tempAvg[NUM_TEMP_READS/2] + TEMP_OFFSET);
}


/* Wake up sleeping main thread following full dma transfer to resume normal processing */
static void dma_tcie_callback(){

	k_wakeup(ia_tid);
}