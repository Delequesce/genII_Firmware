#define DT_DRV_COMPAT analog_ad4002

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/dt-bindings/spi/spi.h>
#include <app/drivers/ad4002.h>
#include <app/drivers/pwm_stm32_custom_trigger.h>
#include <zephyr/irq.h>
#include <stm32_ll_tim.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>

#define LOG_LEVEL CONFIG_AD4002_LOG_LEVEL

LOG_MODULE_REGISTER(analog_ad4002);

/* Device Tree Nodes */
#define AD4002_NODE(index) DT_DRV_INST(index) 
#define PWM_NODE(index)  DT_PWMS_CTLR_BY_IDX(AD4002_NODE(index), 0) // Gets the PWM Node that serves as the convert signal interrupt timer
#define PWM_PARENT_TIMER(index) DT_PARENT(PWM_NODE(index)) // Gets the parent timer of the aforementioned PWM node for interrupt vectoring

/* SPI and PWM Params*/
#define SPI_OPER(index) (SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(SPI_FRAME_BITS))
#define SPI_FRAME_BITS 16
#define CNV_HIGH_TIME 21 // 330 nsec at 64 MHz is 21 clock cycles

struct ad4002_config{
	struct spi_dt_spec spi_spec;
	struct pwm_dt_spec cnv_pwm_spec;
	uint8_t sample_period; 
	void (*irq_config_func)(const struct device *dev);
	struct spi_buf_set *rx_buffer;
};

static int analog_ad4002_continuous_read(const struct device *dev, const struct spi_buf_set *rx_buffer)
{
	int ret; 
	struct ad4002_config *cfg = dev->config;
	cfg->rx_buffer = rx_buffer;
	
	/* Setup PWM CNV Signal */
	/*ret = pwm_set_cycles(cfg->cnv_pwm_spec.dev, cfg->cnv_pwm_spec.channel,cfg->sample_period, CNV_HIGH_TIME, cfg->cnv_pwm_spec.flags);
	if (ret < 0){
		LOG_ERR("Error %d: failed to set convert signal\n", ret);
		return 0;
	}
	return 0;*/
}

/* Internal Function to Start to Read Data following CNV interrupt */
static int analog_ad4002_read(const struct device *dev){

	int ret;
	const struct ad4002_config *cfg = dev->config;
	ret = spi_read_dt(&cfg->spi_spec, cfg->rx_buffer);
	return ret; 
}

static int ad4002_init(const struct device *dev){
	
	const struct ad4002_config *cfg = dev->config;
	
	/* Initialize SPI bus*/
	if (!spi_is_ready_dt(&cfg->spi_spec)) {
		LOG_ERR("SPI device %s not ready", cfg->spi_spec.bus->name);
		return -ENODEV;
	}
	// Set SPI bus clock frequency to frequency in AD4002 node
	//cfg->bus.config.frequency = cfg->clock_speed;

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
        LOG_INF("Capture Compare Interrupt Triggered"); 
		/*analog_ad4002_read(dev);*/
		return;													
	}
	/* TIM Update event */
	if (LL_TIM_IsActiveFlag_UPDATE(timer) && LL_TIM_IsEnabledIT_UPDATE(timer)) {
		LL_TIM_ClearFlag_UPDATE(timer);

        /* Custom update interrupt code */
        LOG_INF("Update Interrupt Triggered And Flags Reset");
		return;
	}
	return;
}																			


static const struct ad4002_driver_api ad4002_api = {
	.read = analog_ad4002_read,
	.continuous_read = analog_ad4002_continuous_read,
};

#define CONFIG_FUNC(index) ad4002_cfg_##index

#define AD4002_DEVICE_INIT(index) 										\
																		\
	static void pwm_irq_config_##index(const struct device *dev)     	\
    {                                                                   \
        IRQ_CONNECT(DT_IRQN(PWM_PARENT_TIMER(index)),				  	\
			    DT_IRQ(PWM_PARENT_TIMER(index), priority),			  	\
			    pwm_irq_handler,				  						\
			    DEVICE_DT_INST_GET(index),				  				\
			    0);							                			\
		irq_enable(DT_IRQN(PWM_PARENT_TIMER(index)));				  	\
    };         															\
																		\
	static const struct ad4002_config ad4002_cfg_##index = { 			\
		.spi_spec = SPI_DT_SPEC_INST_GET(index, SPI_OPER(index), 0), 	\
		.cnv_pwm_spec = PWM_DT_SPEC_INST_GET(index),					\
		.sample_period = DT_PROP(AD4002_NODE(index), sample_period), 	\
		.irq_config_func = pwm_irq_handler								\
	}; 																	\
																		\
DEVICE_DT_INST_DEFINE(index, &ad4002_init, NULL, 						\
	NULL, &ad4002_cfg_##index, POST_KERNEL,      						\
	CONFIG_AD4002_SPI_INIT_PRIORITY, &ad4002_api);

// Initialize each active device
DT_INST_FOREACH_STATUS_OKAY(AD4002_DEVICE_INIT)