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

/* Battery LUTs */
static const uint8_t capacityArray[11] = {100, 94, 85, 75, 62, 53, 40, 22, 13, 3, 0};
//static const uint8_t voltageArray[11] = {4.2, 4.1, 4.0, 3.9, 3.8, 3.7, 3.6, }

/* Function forward Declarations */
static void calculateParameters(circular_buf_t* cbt, uint16_t n, float data,
	struct outputParams* opData, struct calcParamsVars* cpv_local, uint8_t* flags);

/* Main data structure */
static struct impedance_data testDataMat[DEFAULT_EQC_TIME][4] = {{0}};
static float Z_real_Mat[N_AVERAGES] = {0};
static float Z_imag_Mat[N_AVERAGES] = {0};

static const struct impedance_data qcData[5][4] = {
	{{.C = 46.5, .G = 5.57}, {.C = 101.5, .G = 2.57}, {.C = 230.0, .G = 10.05}, {.C = 324.6, .G = 5.02}},
	{{.C = 48.0, .G = 6.688}, {.C = 98.8, .G = 2.570}, {.C = 229.9, .G = 10.080}, {.C = 326.6, .G = 6.686}},
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
static const struct gpio_dt_spec power_button = GPIO_DT_SPEC_GET(POWER_BUTTON, gpios);

// Necessary structure for button interrupt handler
static struct gpio_callback button_cb_data;

//INPUT_CALLBACK_DEFINE(NULL, input_cb, NULL);

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
	#if ASSEMBLY_TESTING
    if (gpio_pin_configure_dt(&adc_shdn_low, GPIO_OUTPUT_ACTIVE) < 0) {
        printk("ESHDN pins not properly configured");
		return 0;
	}
	/* Enable ADC and CC Drive */
	if (pwm_set_cycles(ccDriver.dev, ccDriver.channel, V_SIG_PERIOD, V_SIG_PERIOD/2, ccDriver.flags) < 0){
		printk("EError: Failed to setup CC Drive");
		return -1;
	}
	#else
	if (gpio_pin_configure_dt(&adc_shdn_low, GPIO_OUTPUT_ACTIVE) < 0) {
        printk("ESHDN pins not properly configured");
		return 0;
	}
	#endif

	/* Configure TIA SHDNs, setting all SHDN to start */
    if (!gpio_is_ready_dt(&tia_1_shdn_low) ||
		!gpio_is_ready_dt(&tia_2_shdn_low) ||
		!gpio_is_ready_dt(&tia_3_shdn_low) || 
		!gpio_is_ready_dt(&tia_4_shdn_low)) {
        printk("ETIA SHDN Pins not ready");
		return 0;
	}

	#if ASSEMBLY_TESTING
	if (gpio_pin_configure_dt(&tia_1_shdn_low, GPIO_OUTPUT_ACTIVE) < 0 || 
				gpio_pin_configure_dt(&tia_2_shdn_low, GPIO_OUTPUT_INACTIVE) < 0 || 
				gpio_pin_configure_dt(&tia_3_shdn_low, GPIO_OUTPUT_INACTIVE) < 0 ||
				gpio_pin_configure_dt(&tia_4_shdn_low, GPIO_OUTPUT_INACTIVE) < 0) {
				printk("ETIA Multiplexing Error");
				return 0;
			}
	#else
	if (gpio_pin_configure_dt(&tia_1_shdn_low, GPIO_OUTPUT_INACTIVE) < 0 || 
				gpio_pin_configure_dt(&tia_2_shdn_low, GPIO_OUTPUT_INACTIVE) < 0 || 
				gpio_pin_configure_dt(&tia_3_shdn_low, GPIO_OUTPUT_INACTIVE) < 0 ||
				gpio_pin_configure_dt(&tia_4_shdn_low, GPIO_OUTPUT_INACTIVE) < 0) {
				printk("ETIA Multiplexing Error");
				return 0;
			}
	#endif

	/* Set up power to Raspberry Pi (Power enable) and battery charging (charge enable) GPIOs */
	if (!gpio_is_ready_dt(&charge_enable_high) || gpio_pin_configure_dt(&charge_enable_high, GPIO_OUTPUT_ACTIVE) < 0){
		printk("ECannot set charge enable, Battery Charging disabled");
	}

	if (!gpio_is_ready_dt(&power_enable_low) || gpio_pin_configure_dt(&power_enable_low, GPIO_OUTPUT_INACTIVE) < 0){
		printk("ECannot enable power input to Pi.");
		return 0;
	}
	
	/* Set up power button behavior */
	ret = gpio_is_ready_dt(&power_button);
	if (!ret){
		printk("Error: button device is not ready\n");
	}
	if(ret){
		ret = gpio_pin_configure_dt(&power_button, GPIO_INPUT);
		if(ret != 0){
			printk("EFailed to configure button as input\n");
		}
	}
	if(!ret){
		ret = gpio_pin_interrupt_configure_dt(&power_button, GPIO_INT_EDGE_TO_ACTIVE);
		if (ret != 0) {
			printk("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, power_button.port->name, power_button.pin);
		}
	}
	if(!ret){
		gpio_init_callback(&button_cb_data, button_pressed, BIT(power_button.pin));
		gpio_add_callback(power_button.port, &button_cb_data);
		//printk("Button Initialized!");
	}
	
	/*
	if (readBatteryLevel_Init() < 0){
		printk("EBattery Reading Initialization Failure. Cannot read battery voltages");
	}
	*/

	//k_yield();

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

	
	/* Run Loop to check for flag set by shutdown handler */
	/* We need to disable and re-enable GPIO interrupts to avoid triggering back to back wakeup and shutdowns*/
	for (;;){
		if (activeState == SHUTTINGDOWN){
			ret = gpio_pin_interrupt_configure_dt(&power_button, GPIO_INT_DISABLE);
			shutdownSystem();
			ret = gpio_pin_interrupt_configure_dt(&power_button, GPIO_INT_EDGE_TO_ACTIVE);
		}
		if (activeState ==  WAKINGUP){
			ret = gpio_pin_interrupt_configure_dt(&power_button, GPIO_INT_DISABLE);
			wakeupSystem();
			ret = gpio_pin_interrupt_configure_dt(&power_button, GPIO_INT_EDGE_TO_ACTIVE);
		}
		/* Sleep to allow heater, uart, and IA threads to operate */
		k_msleep(500);
	}

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
						heaterState = (heaterState + 1) % 2;
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
					case 'F': // Debugging System
						activeState = FREERUNNING;
						ia_tid = k_thread_create(&IA_thread_data, IA_stack_area,
										K_THREAD_STACK_SIZEOF(IA_stack_area),
										testThread_entry_point, 
										NULL, NULL, NULL, 
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

	//printk("EHeater Thread Starting\n");

	/* Enable Heater power */
	/*if (gpio_pin_set_dt(&heater_en, 1) < 0) {
		return 0;
	}*/

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

	//static bool heatFlag = 0;

	/* Initial Read */
	tempAvg = readTemp(&sequence);
	CCTemp = tempAvg;
	prevTempAvg = tempAvg;
	prevCCTemp = tempAvg;
	uint8_t tempWriteFlag = 0;

	while(1){
		tempAvg = readTemp(&sequence);

		/* Update estimate of ClotChip Temperature based on rate of change */
		//CCTemp = (tempAvg-prevTempAvg)*1.0f + prevCCTemp; 
		//prevCCTemp = CCTemp;
		//prevTempAvg = tempAvg;

		// for (size_t i = 0U; i < NUM_THERMISTORS; i++){
		// 	if (fabs(channel_temps[i] - tempAvg) > TEMP_DIFF_THRESH){
		// 		printk("ETemperatures on board are spatially uneven\n");
		// 	}
		// }
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
			if (tempAvg < 20){
				heater_errI = 0;
				pulse_cycles = V_SIG_PERIOD;
			}
			else{
				/* Find PID errors and calculate output duty cycle */
				heater_errP = test_cfg.incubationTemp-tempAvg;
				heater_errI = heater_errI + heater_errP;
				heater_errD = prevTempAvg - tempAvg;
				prevTempAvg = tempAvg;
				pulse_cycles = (uint32_t)(V_SIG_PERIOD * (1-(K_P * heater_errP + K_I * heater_errI + K_D * heater_errD)*0.01));
				pulse_cycles = pulse_cycles > V_SIG_PERIOD ? V_SIG_PERIOD:pulse_cycles;
				pulse_cycles = pulse_cycles < 0 ? 0:pulse_cycles;
			}

			if (pwm_set_cycles(heaterPwm.dev, heaterPwm.channel, V_SIG_PERIOD, pulse_cycles, heaterPwm.flags) < 0){
				printk("EError: Failed to set heater pulse");
				return -1;
			}

			char int_buffer[9];
			sprintf(int_buffer, "%lu", pulse_cycles);

			//Optional Print Pulse Cycles
			/*uart_poll_out(uart_dev, 'E');
			for(int k = 0; k < 2; k++){
				uart_poll_out(uart_dev, int_buffer[k]);
			}
			uart_poll_out(uart_dev, '\n');*/
			// Update duty cycle using zephyr driver
			//pulse_cycles = 32;

		}

		/* Schedule new reading every second and let other threads run */
		k_msleep(1000);
	}
	return;
}

/* Performs measurements */
static void testThread_entry_point(const struct test_config* test_cfg, void *unused1, void *unused2){

	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	//printk("EPlaceholder\n");

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

	/*for(uint32_t n = 0; n < SAMPLES_PER_COLLECTION; n++){
        *(Ve_data + n) = 1;
        *(Vr_data + n) = 1;
    } */

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

	/* For Parameter Calculations */
	static struct outputParams opData[N_CHANNELS_MAX] = {{
		.tPeak = 0,
		.deltaEps = 0,
		.deltaEpsTime = 0,
		.smax = 0,
		.smaxTime = 0,
	}};

	static struct calcParamsVars cpv[N_CHANNELS_MAX] = {{
		.prevX = 0,
		.C_max = 0,
		.x_ma = 0,
		.slp = 0,
	}};

	static struct dataWriteStruct dwStruct[N_CHANNELS_MAX];

	static float ma_buf0[MA_BUF_N];
    static float ma_buf1[MA_BUF_N];
    static float ma_buf2[MA_BUF_N];
    static float ma_buf3[MA_BUF_N];
	static circular_buf_t cbt[N_CHANNELS_MAX] = {{.buffer = ma_buf0}, {.buffer = ma_buf1}, {.buffer = ma_buf2}, {.buffer = ma_buf3}};
    for(i = 0; i < N_CHANNELS_MAX; i++){
	    circular_buffer_init(&cbt[i], MA_BUF_N);
    }
    uint8_t flags[N_CHANNELS_MAX] = {0};

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
	#if USE_REAL_DATA
	ad4002_init_read(ad4002_master, ad4002_slave, Ve_data, Vr_data, SAMPLES_PER_COLLECTION);
	ad4002_irq_callback_set(ad4002_master, &dma_tcie_callback);
	#endif
	volatile int64_t sleepTime, timeStamp; // Timing params for measuring speed

	/* Timing Parameters */
	int64_t startTime = k_uptime_get();

	/* Set to continuous run if FREERUNNING enabled for debugging */
	if (activeState == FREERUNNING){
		printk("Now Entering Free Run Debug Mode\n");
		/* Set Channel 1 to always on */
		c = 0;
		if (gpio_pin_configure_dt(&tia_1_shdn_low, tia_shdn_states[c+3]) < 0 || 
				gpio_pin_configure_dt(&tia_2_shdn_low, tia_shdn_states[c+2]) < 0 || 
				gpio_pin_configure_dt(&tia_3_shdn_low, tia_shdn_states[c+1]) < 0 ||
				gpio_pin_configure_dt(&tia_4_shdn_low, tia_shdn_states[c]) < 0) {
				printk("ETIA Multiplexing Error");
				return 0;
			}

		while(true){

			/* Read Data from ADC */
			#if USE_REAL_DATA
			ad4002_start_read(ad4002_master, SAMPLES_PER_COLLECTION);
			k_msleep(2); // Thread sleeps until DMA callback is triggered

			//t2 = k_uptime_get();
			/* Copy data to safe memory location */ 
			memcpy(Ve_data_safe, Ve_data, SAMPLES_PER_COLLECTION*2);
			memcpy(Vr_data_safe, Vr_data, SAMPLES_PER_COLLECTION*2);
			k_msleep(1);
			#endif

		}
		return;
	}

	/* This loop runs each collection for the entire test run time (outer loop) */
	for(i = 0; i < N_Measurements; i++){

		/* This loop runs to obtain repeat measurements over the collection frequency interval */
		//t1 = k_uptime_get();

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
			#if USE_REAL_DATA
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
				/* Final Step calculates complex FFT bin for each signal */
				Ve_real = 3*(cos_w0 * prev_value_Ve - prev_prev_value_Ve)/N_FFT/65535.0f;
				Ve_imag = 3*(sin_w0 * prev_value_Ve)/N_FFT/65535.0f;

				Vr_real = 3*(cos_w0 * prev_value_Vr - prev_prev_value_Vr)/N_FFT/65535.0f;
				Vr_imag = 3*(sin_w0 * prev_value_Vr)/N_FFT/65535.0f;

				/* Finally, compute Z */
				Z_temp_real = COMPLEX_DIVIDE_REAL(Ve_real, Ve_imag, Vr_real, Vr_imag);
				Z_temp_imag = COMPLEX_DIVIDE_IMAG(Ve_real, Ve_imag, Vr_real, Vr_imag);
				//Z_real = (Z_real*j + COMPLEX_MULTIPLY_REAL(Z_temp_real, Z_temp_imag, calibMat[c].Zfb_real, calibMat[c].Zfb_imag))/(j+1); // Moving Average
				//Z_imag = (Z_imag*j + COMPLEX_MULTIPLY_IMAG(Z_temp_real, Z_temp_imag, calibMat[c].Zfb_real, calibMat[c].Zfb_imag))/(j+1);
				
				// Full Set for Median Calculation or Outlier Removal
				Z_real_Mat[j] = COMPLEX_MULTIPLY_REAL(Z_temp_real, Z_temp_imag, calibMat[c].Zfb_real, calibMat[c].Zfb_imag);
				Z_imag_Mat[j] = COMPLEX_MULTIPLY_IMAG(Z_temp_real, Z_temp_imag, calibMat[c].Zfb_real, calibMat[c].Zfb_imag);
			}
		
			/* Median Calculation */
			// Q Sort Based
			qsort(Z_real_Mat, N_AVERAGES, sizeof(float), compare);
			qsort(Z_imag_Mat, N_AVERAGES, sizeof(float), compare);
			Z_real = Z_real_Mat[N_AVERAGES/2];
			Z_imag = Z_imag_Mat[N_AVERAGES/2];

			/* Apply small real and imaginary Z Offsets */
			Z_real += Z_OFF_REAL;
			Z_imag += Z_OFF_IMAG;
			
			/* Update calibration moving average */
			if (activeState == CALIBRATING){
				Z_real_mean[c] = (Z_real_mean[c] * i + Z_real)/(i+1);
				Z_imag_mean[c] = (Z_imag_mean[c] * i + Z_imag)/(i+1);
				//printk("EZ_real: %0.4f\n", Z_real_mean);
				//printk("EZ_imag: %0.4f\n", Z_imag_mean);
			}
			else{
				/* Final Calculation and storage */
				mag2Z = Z_real*Z_real + Z_imag*Z_imag;
				if(activeState == EQC)
				{
					testDataMat[i][c].G = 1000 * Z_real/mag2Z;				// Result is in mS
					testDataMat[i][c].C = 159154.943091895f * Z_imag/mag2Z; // magic number is 1e12 / (2*pi*1e6). Result is in pF
				}
				else
				{
					testDataMat_lite[c].G = 1000 * Z_real/mag2Z;				// Result is in mS
					testDataMat_lite[c].C = 159154.943091895f * Z_imag/mag2Z; // magic number is 1e12 / (2*pi*1e6). Result is in pF
					/* Function to calculate Output Parameters and moving average */
					calculateParameters(&cbt[c], i, testDataMat_lite[c].C, &opData[c], &cpv[c], &flags[c]);
					testDataMat_lite[c].C = cpv[c].x_ma;

					/* Package into single data structure */
					dwStruct[c].impDat = testDataMat_lite[c];
					dwStruct[c].opDat = opData[c];
				}
			}
			#else 

			k_msleep(100);

			/* Code to execute if using fake data saved to MCU memory*/
			testDataMat_lite[c].G = 0;
			testDataMat_lite[c].C = CDataFake[i][c];

			calculateParameters(&cbt[c], i, testDataMat_lite[c].C, &opData[c], &cpv[c], &flags[c]);
					testDataMat_lite[c].C = cpv[c].x_ma;

			/* Package into single data structure */
			dwStruct[c].impDat = testDataMat_lite[c];
			dwStruct[c].opDat = opData[c];

			#endif
		}

		/* Send data over uart */
		if (activeState == TESTRUNNING){

			/* Total write at 115200 baud should take 8-10 msec */
			uart_write_32f(&dwStruct[0], 28, 'D');

		}

		//ad4002_shutdown(ad4002_master);
		/* Collection timestamp */
		#if REAL_TIME
		timeStamp = k_uptime_get() - startTime;

		/* Sleep until next collection period */
		sleepTime = (test_cfg->collectionInterval) * 1000*(i+1) - timeStamp;
		#else
		sleepTime = 300;
		#endif
		
		k_msleep(sleepTime); // Usually around 270 msec
		//t2 = k_uptime_get();
		//printk("TTotal loop time: %lli\n", t2-t1);
		//k_msleep(1000);
	}

	/* Perform additional calibration steps, if necessary */
	if(activeState == CALIBRATING){
		// Rev 2
		//const float test_Z_real = 179.636; 
		//const float test_Z_imag = 0.043; 
		// Rev 3
		const float test_Z_real = 149.477; 
		const float test_Z_imag = 0.070;

		const float expected_calib_real = 63;
		const float expected_calib_imag = 20;

		float div_temp_real;
		float div_temp_imag;

		/* Read current calibration values as reference */
		flash_read(flash_device, PAGE200, &calibMat, 32);

		for(c=0; c<4; c++){
			if (test_cfg->channelOn[c]){
				div_temp_real = COMPLEX_DIVIDE_REAL(test_Z_real, test_Z_imag, Z_real_mean[c], Z_imag_mean[c]);
				div_temp_imag = COMPLEX_DIVIDE_IMAG(test_Z_real, test_Z_imag, Z_real_mean[c], Z_imag_mean[c]);

				/* Check if it matches what is expected before storing */
				if (fabs(div_temp_real - expected_calib_real) < 5 && fabs(div_temp_imag - expected_calib_imag) < 30){
					calibMat[c].Zfb_real = div_temp_real;
					calibMat[c].Zfb_imag = div_temp_imag;
				}
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
	else if(activeState == EQC){
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
	else{
		/* Store Test Data in Flash */
		// To Do...

	}

	/* Tell UI that test has completed */
	activeState = IDLE;
	uart_write_singleChar('X', true);
	return; 
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
	//float m_temp = 0.0349;
	//float b_temp = -13.4;

	/* Axial Thermistors (R25 = 10000, B = 3950, R = 8000)*/
	float m_temp = 0.0287;
	float b_temp = -17.8;

	float channel_temps_local[NUM_THERMISTORS];
	channel_temps_local[0] = 0;
	float tempAvg[5] = {0, 0, 0, 0, 0};
	for (uint8_t j = 0; j < 5; j++){
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
		}

		for (uint8_t i = 0; i < NUM_THERMISTORS; i++){
			tempAvg[j] = (channel_temps_local[i] + tempAvg[j] * i)/(i+1);
		}
	}

	// Mean
	// Median
	qsort(tempAvg, 5, sizeof(float), compare);

	return tempAvg[2] - TEMP_OFFSET;
}


/* Wake up sleeping main thread following full dma transfer to resume normal processing */
static void dma_tcie_callback(){

	k_wakeup(ia_tid);
}

/** Sets up battery level read, either should make into thread or call on startup */
static uint8_t readBatteryLevel_Init(){

	/* Buffer where samples will be written */
	uint16_t buf;
	struct adc_sequence sequence = {
		.buffer = &buf,
		/* buffer size in bytes, not number of samples */
		.buffer_size = sizeof(buf),
		.calibrate = false,
	};
	
	/* Configure channel and sequence prior to sampling. */
	if (!adc_is_ready_dt(&adc_channels[NUM_THERMISTOR_CHANNELS])) {
		printk("EADC controller device %s not ready\n", adc_channels[NUM_THERMISTOR_CHANNELS].dev->name);
		return -1;
	}

	if (adc_channel_setup_dt(&adc_channels[NUM_THERMISTOR_CHANNELS]) < 0) {
		printk("ECould not setup battery read channel\n");
		return -1;
	}

	return 0;
	/* End Initialization Block */
}

/** This function is called periodically to read the battery level and send data to Pi */
static uint8_t readBatteryLevel(struct adc_sequence* sequence){

	int err;
	int16_t* val_mv_ptr;
	int32_t val_mv;
	uint8_t currentBatteryCapacity = 0;

	/* First have to disable battery charging to allow for reading voltage */
	if (gpio_pin_configure_dt(&charge_enable_high, GPIO_OUTPUT_INACTIVE) < 0) {
        printk("ECannot Read Battery Level\n");
		return -1;
	}

	(void)adc_sequence_init_dt(&adc_channels[NUM_THERMISTOR_CHANNELS], sequence);

	err = adc_read_dt(&adc_channels[NUM_THERMISTOR_CHANNELS], sequence);
	if (err < 0) {
		printk("ECould not read (%d)\n", err);
		return -1;
	}
	
	val_mv_ptr = sequence->buffer; // Can't dereference a generic pointer, so have to cast to int16_t
	val_mv = (int32_t)(*val_mv_ptr & 0xFFFF); // Cast the 16b data to 32b. This is the voltage level in mV
	err = adc_raw_to_millivolts_dt(&adc_channels[NUM_THERMISTOR_CHANNELS], &val_mv);
	if (err < 0) {
		printk("EValue in mV not available\n");
		return -1;
	}
	
	/**
	 * Compare to LUT to determine battery level 
	 * If the battery is actively supplying power, will have to take into account cabling resistance 
	 * and average current level. Need to take readings over a 5-10 second period and then average.
	 * If the battery is being charged, this will not need to be taken into account, since the discharge rate
	 * is 0. Therefore, we will need 2 LUTs. 
	 * */
	currentBatteryCapacity = capacityArray[(int8_t)(0.01 * (4200 - val_mv))];

	/* Finally re-enable battery charging */
	if (gpio_pin_configure_dt(&charge_enable_high, GPIO_OUTPUT_ACTIVE) < 0) {
        printk("EError Re-enabling battery charging\n");
		return -1;
	}
	return currentBatteryCapacity;
}

/**
 * These are called when a specific interrupt is triggerred
 * The procedure is below: 
 * 1) User holds button down. This triggers ISR on PC5.
 * 2) The ISR determines if its a wakeup or shutdown by checking status of power_enable pin. 
 *    It then calls the relevant function: wakeupSystem() or shutdownSystem()
 * SHUTDOWN
 * 1) Shutdown function aborts test and heater threads, keeping uart for communication with Pi. 
 * 2) It then sends UART signal to Pi with shutdown command (b'ZZZ').
 * 3) It then idles and waits for response b'K' from Pi, indicating that it is shutting itself down
 * 4) MCU waits 30 seconds before toggling POWER_EN to HIGH, cutting power to Pi. 
 * 5) Finally, the MCU shuts down the uart thread and just waits for second ISR on PC5 (button press) 
 *    to wake system up.
 * WAKEUP
 * 1) Wakeup function starts up the heater and uart threads as is done on system reset. Also needs to reset 
 *    all GPIOs to their default state
 * 2) Also toggles POWER_EN to power and start up Pi. 
 * */


/* Called from interrupt handler. Cannot sleep because not backed by execution thread */
static void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
	/* Either shutdown or wakeup system */
	//printk("Button Pressed\n");
	if (activeState == POWEREDOFF){
		activeState = WAKINGUP;
		return;
	}
	if (activeState == WAKINGUP){
		return;
	}
	/* Default case */
	activeState = SHUTTINGDOWN;
	return;
}

static void wakeupSystem(){
	printk("ZWaking up system\n");
	if (gpio_pin_configure_dt(&power_enable_low, GPIO_OUTPUT_INACTIVE) < 0){
		printk("Error Re-enabling Power");
		return;
	}

	/* Need to re=initialize system as if it was booting from the beginning */

	/* Create Default Threads */
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
	
	activeState = IDLE;
	heaterState = NOT_HEATING;
	return;
}


static void shutdownSystem(){
	/* Transmit shutdown signal to Pi */
	printk("ZShutting down System\n");
	printk("ZZZZ\n");

	/* Wait 15 seconds for shutdown and then stop power. */
	k_busy_wait(POWER_OFF_TIME_MS);

	/* Shut down active threads */
	if (activeState == TESTRUNNING){
		k_thread_abort(ia_tid);
		activeState = IDLE;
	}
	k_thread_abort(heater_tid);
	k_thread_abort(uartio_tid);

	if (gpio_pin_configure_dt(&power_enable_low, GPIO_OUTPUT_ACTIVE) < 0){
		printk("Error Disabling Power");
		activeState = IDLE;
		return;
	}

	/* Configure button interrupt as wakeup source */
	//gpio_pin_configure_dt(&power_button, STM32_GPIO_WKUP);

	/* Go into sleep but maintain power regulators.*/
	pwm_set_cycles(heaterPwm.dev, heaterPwm.channel, V_SIG_PERIOD, V_SIG_PERIOD, heaterPwm.flags);
	deviceConnected = false;

	/* Actual system poweroff. */
	printk("System powered down\n");
	//sys_poweroff();
	activeState = POWEREDOFF;

	return;
}

// static void input_cb(struct input_event *evt, void *user_data){
// 	ARG_UNUSED(user_data);
// 	switch (evt->code){
// 		case INPUT_KEY_A:
// 			printk("Short Press");
// 			break;
// 		case INPUT_KEY_X:
// 			printk("Long Press");
// 			break;
// 		default:
// 			printk("Unknown Event");
// 			return;
// 	}
// }


/**
 * Runs every iteration of data collection to calculate parameters
 * flags is [slp_flag, tPeakFound, deltaEpsFound, smaxFound]
 * */

static void calculateParameters(circular_buf_t* cbt, uint16_t n, float data, struct outputParams* opData_local, struct calcParamsVars* cpv_local, uint8_t* flags)
{
	// Add incoming data to moving average window
	circular_buffer_put(cbt, data);
	cpv_local->x_ma = circular_buffer_avg(cbt);

	/* Not sure if necessary */
	if(n < 1)
	{
		cpv_local->prevX = 0;

	}

	// Don't calculate parameters until after 60 seconds
	if (n < EARLIEST_PEAK_TIME){
		return;
	}

	cpv_local->slp = cpv_local->x_ma - cpv_local->prevX;
	cpv_local->prevX = cpv_local->x_ma;


	/* Tpeak */
	if(!PEAKFOUND(flags)){
		if(!SLPFLAG(flags) && cpv_local->slp > 0){
			(*flags) ^= SLPFLAG_I;
			return;
		}
		
		if(cpv_local->slp < 0 && opData_local->tPeak == 0)
		{
			// Local or Absolute Maximum
			opData_local->tPeak = n;
			cpv_local->C_max = data;
			return;
		}

		// Determine if max is local or absolute
		if(data > cpv_local->C_max)
		{
			// Can't be a maximum, reset tpeak and Cmax
			opData_local->tPeak = 0;
			cpv_local->C_max = 0;
			return;
		}
		(*flags) ^= PEAKFOUND_I;
		//printf("Peak Found at t = %d sec, %0.2f pF\n", opData->tPeak, C_max);
		return;
	}
	/* Smax (Un-normalized) */
	if(!SMAXFOUND(flags))
	{
		//printf("%d: slp = %0.8f\t smax = %0.8f\t", n, slp, smax);
		if(cpv_local->slp < opData_local->smax){
			opData_local->smax = cpv_local->slp;
			opData_local->smaxTime = n;
		}
	}

	/* Delta Epsilon (Un-normalized)*/
	if(!DELTAEPSFOUND(flags)){
		// Wait until value has fallen significantly from the peak
		if(cpv_local->x_ma > cpv_local->C_max * FALL_THRESH){
			return;
		}
		if(cpv_local->slp > SLOPE_THRESH*cpv_local->C_max)
		{
			opData_local->deltaEps = 1-(data/cpv_local->C_max);
			opData_local->deltaEpsTime = n;
			(*flags) ^= DELTAEPSFOUND_I;
			//printf("Delta Eps Found at t = %d sec, %0.4f\n", opData->deltaEpsTime, opData->deltaEps);

			opData_local->smax = opData_local->smax/cpv_local->C_max;
			(*flags) ^= SMAXFOUND_I;
			//printf("Smax Found at t = %d sec, %0.6f\n", opData->smaxTime, opData->smax);

			return;
		}
	}
}