#define DT_DRV_COMPAT analog_ad4002

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/dt-bindings/spi/spi.h>
#include <zephyr/sys/printk.h>
#include <app/drivers/ad4002.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/pwm.h>
#include <stm32_ll_tim.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>

//LOG_MODULE_REGISTER(analog_ad4002, CONFIG_AD4002_LOG_LEVEL);

/* Device Tree Nodes */
#define AD4002_NODE(index) DT_DRV_INST(index) 
#define PWM_NODE(index)  DT_PWMS_CTLR_BY_IDX(AD4002_NODE(index), 0) // Gets the PWM Node that serves as the convert signal interrupt timer
#define PWM_PARENT_TIMER(index) DT_PARENT(PWM_NODE(index)) // Gets the parent timer of the aforementioned PWM node for interrupt vectoring

/* SPI and PWM Params*/
#define SPI_OPER(index) 	(SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(SPI_FRAME_BITS))
#define SPI_FRAME_BITS 		16
#define CNV_LOW_TIME 		41 // 330 nsec at 64 MHz is 21 clock cycles, but needs to be flipped due to polarity shift
#define RX_BUFFER_LENGTH    2

/* DMA Macros */
#define SPI_DMA_CHANNEL(node)							\
	.dma = {											\
		COND_CODE_1(DT_DMAS_HAS_NAME(node, rx),			\
			(SPI_DMA_CHANNEL_INIT(node, rx)),			\
			(NULL))										\
		},

struct ad4002_config{
	struct spi_dt_spec spi_spec;
	struct pwm_dt_spec cnv_pwm_spec;
	uint8_t sample_period; 
	void (*irq_config_func)(const struct device *dev);
	struct spi_buf_set *rx_buffer;
};

struct ad4002_data{
	struct spi_buf_set rx_buffer;
};

static int analog_ad4002_continuous_read(const struct device *dev)
{
	int ret; 
	const struct ad4002_config *cfg = dev->config;
	
	/* Setup PWM CNV Signal */
	ret = pwm_set_cycles(cfg->cnv_pwm_spec.dev, cfg->cnv_pwm_spec.channel,cfg->sample_period, CNV_LOW_TIME, cfg->cnv_pwm_spec.flags);
	if (ret < 0){
		printk("Error %d: failed to set convert signal\n", ret);
		return -1;
	}
	return 0;
}

/* Sets up and returns an appropriate SPI buffer for read operations */
static struct spi_buf_set setup_spi_rx_buffer(){
	
    uint8_t rx_buffer[RX_BUFFER_LENGTH] = {0};
	struct spi_buf my_spi_buffer[1];
	my_spi_buffer[0].buf = rx_buffer;
	my_spi_buffer[0].len = RX_BUFFER_LENGTH;
	const struct spi_buf_set rx_buff = {my_spi_buffer, 1};

	return rx_buff;
}


/* Internal Function to Start to Read Data following CNV interrupt */
static void analog_ad4002_read(const struct device *dev){

	int ret;
	const struct ad4002_config *cfg = dev->config;
	spi_read_dt(&cfg->spi_spec, cfg->rx_buffer);
	printk("Data obtained: %d", (*cfg->rx_buffer));
}

static int ad4002_init(const struct device *dev){
	
	const struct ad4002_config *cfg = dev->config;
	struct ad4002_data *dev_data = dev->data;
	dev_data->rx_buffer = setup_spi_rx_buffer();
	
	/* Initialize SPI bus*/
	if (!spi_is_ready_dt(&cfg->spi_spec)) {
		printk("SPI device %s not ready", cfg->spi_spec.bus->name);
		return -ENODEV;
	}

	/* Configure and Enable PWM IRQ */
	cfg->irq_config_func(dev);

	return 0;
}

/** PWM configuration. */
struct pwm_stm32_config {
	TIM_TypeDef *timer;
	uint32_t prescaler;
	uint32_t countermode;
	struct stm32_pclken pclken;
	const struct pinctrl_dev_config *pcfg;
};

/* Timer Style Low Latency Interrupt Handler */
void pwm_irq_handler(const struct device *dev)
{
	/* Obtain timer object from PWM device configuration structure */
	const struct ad4002_config *cfg = dev->config;
	const struct device *pwm_device = cfg->cnv_pwm_spec.dev;
	const struct pwm_stm32_config *pwm_cfg = pwm_device->config;
	TIM_TypeDef *timer = pwm_cfg->timer;

	/* Compare Capture Events */
	if (LL_TIM_IsActiveFlag_CC1(timer) && LL_TIM_IsEnabledIT_CC1(timer)) {
            LL_TIM_ClearFlag_CC1(timer);		
        /* Custom CC Interrupt Code */   							
        printk("Capture Compare Interrupt Triggered"); 
		//analog_ad4002_read(dev);
		return;													
	}
	/* TIM Update event */
	if (LL_TIM_IsActiveFlag_UPDATE(timer) && LL_TIM_IsEnabledIT_UPDATE(timer)) {
		LL_TIM_ClearFlag_UPDATE(timer);

        /* Custom update interrupt code */
        printk("Update Interrupt Triggered And Flags Reset");
		return;
	}
	return;
}																			

static const struct ad4002_driver_api ad4002_api = {
	.read = analog_ad4002_read,
	.continuous_read = analog_ad4002_continuous_read,
};

#define IRQ_CONFIG_FUNC(index)											\
static void pwm_irq_config_##index(const struct device *dev)     		\
    {                                                                   \
        IRQ_CONNECT(DT_IRQN(PWM_PARENT_TIMER(index)),				  	\
			    DT_IRQ(PWM_PARENT_TIMER(index), priority),			  	\
			    pwm_irq_handler,				  						\
			    DEVICE_DT_INST_GET(index),				  				\
			    IRQ_ZERO_LATENCY);							            \
		irq_enable(DT_IRQN(PWM_PARENT_TIMER(index)));				  	\
    };  

#define AD4002_DEVICE_INIT(index) 										\
																		\
	IRQ_CONFIG_FUNC(index)  											\
																		\
	static const struct ad4002_config ad4002_cfg_##index = { 			\
		.spi_spec = SPI_DT_SPEC_INST_GET(index, SPI_OPER(index), 0), 	\
		.cnv_pwm_spec = PWM_DT_SPEC_INST_GET(index),					\
		.sample_period = DT_PROP(AD4002_NODE(index), sample_period), 	\
		.irq_config_func = pwm_irq_config_##index,						\
	}; 																	\
																		\
	static struct ad4002_data ad4002_dev_data_##index = {				\
		.rx_buffer = NULL,												\
	};																	\
																		\
DEVICE_DT_INST_DEFINE(index, &ad4002_init, NULL, 						\
	NULL, &ad4002_cfg_##index, POST_KERNEL,      						\
	CONFIG_AD4002_SPI_INIT_PRIORITY, &ad4002_api);

// Initialize each active device
DT_INST_FOREACH_STATUS_OKAY(AD4002_DEVICE_INIT)