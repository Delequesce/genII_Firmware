#ifndef MAIN_INCLUDE_HEADER
#define MAIN_INCLUDE_HEADER

/* Configuration flags */
#define ASSEMBLY_TESTING			0

/* DT NODELABELS */
#define CCDRIVER	        		DT_ALIAS(my_ccdrive)
#define AD4002_INSTANCE_1   		DT_ALIAS(ad4002_ch1)
#define AD4002_INSTANCE_2   		DT_ALIAS(ad4002_ch2)
#define ADC_SHDN_LOW        		DT_ALIAS(my_adc_shdn_low)
#define TIA1_SHDN_LOW				DT_ALIAS(my_tia1_shdn_low)
#define TIA2_SHDN_LOW				DT_ALIAS(my_tia2_shdn_low)
#define TIA3_SHDN_LOW				DT_ALIAS(my_tia3_shdn_low)
#define TIA4_SHDN_LOW				DT_ALIAS(my_tia4_shdn_low)

#define CHARGE_ENABLE_HIGH			DT_ALIAS(my-charge-enable-high)
#define POWER_ENABLE_LOW			DT_ALIAS(my-power-enable-low)
//#define POWER_BUTTON				DT_ALIAS(my-power-button)

#define HEATERPWM	        		DT_ALIAS(my_heaterpwm)
#define K_P							8 // Ku = 25
#define K_I							0.1 // Pu = 40 sec
#define K_D							20 // May not be necessary

/* Threading Params */
#define IA_THREAD_PRIORITY         	2    // Adjust as needed
#define UARTIO_THREAD_PRIORITY   	4
#define HEATER_THREAD_PRIORITY		5
#define IA_STACK_SIZE            	4096
#define UARTIO_STACK_SIZE        	1024
#define HEATER_STACK_SIZE			1024
//#define N_DATA_BYTES             	8 // C (4), G (4) = 8 bytes

/* UART */
#define MSG_SIZE 9

/* Measurement params */
#define SAMPLES_PER_COLLECTION  	1050
#define N_FFT						1024
#define N_AVERAGES					100
#define SLEEP_TIME_MS 				4
#define DEFAULT_COLLECTION_INTERVAL	1
#define DEFAULT_CALIBRATION_TIME	10
#define DEFAULT_RUN_TIME			1800
#define DEFAULT_EQC_TIME			30
#define DEFAULT_INCUBATION_TEMP		37
#define DEFAULT_SPOT_FREQUENCY		1000000
#define CONVERT_FREQUENCY			1032258
#define W0							0.19634916
#define DEFAULT_ZFB_REAL			1
#define DEFAULT_ZFB_IMAG			0
#define Z_OFF_REAL					0
#define Z_OFF_IMAG					0
#define ADC_RESOLUTION				16
#define BITS_USED					16
#define RESOLUTION_MASK				0xFFFF << (ADC_RESOLUTION - BITS_USED) // 0xFFFF is 16 bits, 0xFFFC is 14
#define MAX_N_MEASUREMENTS			0x1000 // (0x1000 = 4096)
#define NUM_THERMISTOR_CHANNELS		2

/* Other constants */
#define PI						 	3.141592654
#define V_SIG_PERIOD        		64  /* In clock cycles */
//#define N_DATA_BYTES				4 
#define PAGE255						0x0FF000
#define PAGE200						0x0C8200
#define UART_LSB_FIRST				0
#define UART_MSB_FIRST				1
#define NUM_THERMISTORS				2
#define TEMP_DIFF_THRESH			1 // Difference in degrees C allowed between any two thermistor readings
#define TEMP_OFFSET					-1

/* For calculate parameters */
#define MA_BUF_N 			10
#define SLOPE_THRESH		-0.00002
#define FALL_THRESH			0.99
#define EARLIEST_PEAK_TIME	10 // Usually
#define SLPFLAG_I			0x01
#define PEAKFOUND_I			0x02
#define DELTAEPSFOUND_I		0x04
#define SMAXFOUND_I			0x08
#define SLPFLAG(x)			((*x) & SLPFLAG_I)
#define PEAKFOUND(x)		((*x) & PEAKFOUND_I)
#define DELTAEPSFOUND(x)	((*x) & DELTAEPSFOUND_I)
#define SMAXFOUND(x)		((*x) & SMAXFOUND_I)


/* Helper Macros */
#define COMPLEX_DIVIDE_REAL(r1, i1, r2, i2) (r1*r2 + i1*i2)/(r2*r2 + i2*i2)
#define COMPLEX_DIVIDE_IMAG(r1, i1, r2, i2) (r2*i1 - r1*i2)/(r2*r2 + i2*i2)
#define COMPLEX_MULTIPLY_REAL(r1, i1, r2, i2) (r1*r2 - i1*i2)
#define COMPLEX_MULTIPLY_IMAG(r1, i1, r2, i2) (r2*i1 + r1*i2)
#define COMPARE(a, b) (((a) > (b)) - ((a) < (b)))

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

/* Structs, Unions, and Enums */
enum testStates {
	IDLE,
	TESTRUNNING,
	CALIBRATING,
	EQC,
};

enum heaterStates {
	NOT_HEATING,
	HEATING
};

struct test_config{
	uint16_t runTime; // Test run time in seconds
	uint8_t collectionInterval; // Interval in seconds between measurements
	uint8_t incubationTemp; // Incubation Temperature 
	bool channelOn[4]; // Boolean array indicating which channels are active
};

struct calibration_data{
	float Zfb_real;
	float Zfb_imag;
};

struct impedance_data{
	float C;
	float G;
};

struct outputParams{
	float tPeak;
	float deltaEps;
	float deltaEpsTime;
	float smax;
	float smaxTime;
};


/* Other global Variables */
static float heater_errI;
static bool deviceConnected = false;

/* Function Forward Declaration */
static int configure_uart_device(const struct device *dev);
static void uartIOThread_entry_point();
static void testThread_entry_point(const struct test_config* test_cfg, void *unused1, void *unused2);
static void heaterThread_entry_point(void *unused1, void *unused2, void *unused3);
static int stopTest();
static void uart_write_32f(float* data, uint8_t numData, char messageCode);
static float readTemp(struct adc_sequence* sequence);
static void dma_tcie_callback();
static uint8_t readBatteryLevel(struct adc_sequence* sequence);
static uint8_t readBatteryLevel_Init();

#endif