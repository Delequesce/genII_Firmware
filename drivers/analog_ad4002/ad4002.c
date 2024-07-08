#define DT_DRV_COMPAT analog_ad4002

#include <zephyr/device.h>
#include <zephyr/kernel.h>
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
#include <zephyr/drivers/dma/dma_stm32.h>
#include <zephyr/drivers/dma.h>

//LOG_MODULE_REGISTER(analog_ad4002, CONFIG_AD4002_LOG_LEVEL);

/* Device Tree Nodes */
#define AD4002_NODE(index) DT_DRV_INST(index) 
#define PWM_NODE(index)  DT_PWMS_CTLR_BY_IDX(AD4002_NODE(index), 0) // Gets the PWM Node that serves as the convert signal interrupt timer
#define PWM_PARENT_TIMER(index) DT_PARENT(PWM_NODE(index)) // Gets the parent timer of the aforementioned PWM node for interrupt vectoring
#define SPI_NODE(index) DT_BUS(AD4002_NODE(index))

/* SPI and PWM Params*/
#define SPI_OPER(index) 	(SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(SPI_FRAME_BITS))
#define SPI_FRAME_BITS 		16
#define CNV_HIGH_TIME 		21 // 21 Clock cycles is about 330 nsec at 64 MHz
#define RX_BUFFER_LENGTH    2

/* DMA Macros */
#define DMA_CHANNEL_CONFIG(node, dir)	    DT_DMAS_CELL_BY_NAME(node, dir, channel_config)
#define SPI_DMA_CHANNEL_INIT(node, dir)				\
	.dev = DEVICE_DT_GET(DT_DMAS_CTLR(node)),			\
	.channel = DT_DMAS_CELL_BY_NAME(node, dir, channel),		\
	.reg = (DMA_TypeDef *)DT_REG_ADDR(				\
				   DT_PHANDLE_BY_NAME(node, dmas, dir)),\
	.pclken = {                                                             \
		.enr = DT_CLOCKS_CELL(DT_DMAS_CTLR(node), bits),                \
		.bus = DT_CLOCKS_CELL(DT_DMAS_CTLR(node), bus),                 \
	},  \
	.cfg = {							\
		.dma_slot = DT_DMAS_CELL_BY_NAME(node, dir, slot),	\
		.source_data_size = 1, \
		.dest_data_size = 1, \
		.channel_priority = STM32_DMA_CONFIG_PRIORITY(		\
					DMA_CHANNEL_CONFIG(node, dir)), \
		.cyclic = ((DMA_CHANNEL_CONFIG(node, dir) >> 5) & 0x1), \
		.source_burst_length = 1, \
		.dest_burst_length = 1, \
	},								\
	.src_addr_increment = STM32_DMA_CONFIG_PERIPHERAL_ADDR_INC(DMA_CHANNEL_CONFIG(node, dir)), \
	.dst_addr_increment = STM32_DMA_CONFIG_MEMORY_ADDR_INC(DMA_CHANNEL_CONFIG(node, dir)),

#define SPI_DMA_CHANNEL(node)							\
	.dma = {											\
		COND_CODE_1(DT_DMAS_HAS_NAME(node, rx),			\
			(SPI_DMA_CHANNEL_INIT(node, rx)),			\
			(NULL))										\
		},

struct stream {
	DMA_TypeDef *reg;
	const struct device *dev;
	uint32_t channel;
	struct dma_config cfg;
	struct stm32_pclken pclken;
	bool src_addr_increment;
	bool dst_addr_increment;
};

uint64_t startTime;
uint16_t numSamples;

/* Read only configuration data set at build time*/
struct ad4002_config{
	SPI_TypeDef* spi_block;
	struct pwm_dt_spec cnv_pwm_spec;
	uint8_t sample_period; 
	void (*irq_config_func)(const struct device *dev);
	struct stream dma; 
};

/* Mutable data, stored in RAM */
struct ad4002_data{
	uint32_t N_samples;
};

/** PWM configuration. */
struct pwm_stm32_config {
	TIM_TypeDef *timer;
	uint32_t prescaler;
	uint32_t countermode;
	struct stm32_pclken pclken;
	const struct pinctrl_dev_config *pcfg;
};

static int analog_ad4002_continuous_read(const struct device *dev, int16_t* rx_buffer, const uint32_t N_samples)
{
	int ret; 
	const struct ad4002_config *cfg = dev->config;
	const struct device *pwm_device = cfg->cnv_pwm_spec.dev;
	const struct pwm_stm32_config *pwm_cfg = pwm_device->config;
	struct ad4002_data *dev_data = dev->data;
	dev_data->N_samples = N_samples;


	/* Set DMA memory location */
	struct stream *dma_stream = &(cfg->dma);
	DMA_Channel_TypeDef* dma_block = dma_stream->reg + 1; 
	dma_block = dma_block + dma_stream->channel -1;

	dma_block->CMAR = rx_buffer;

	/* Enable SPI */
	SPI_TypeDef* spi_block = cfg->spi_block;
	SET_BIT(spi_block->CR1, SPI_CR1_SPE);
	
	/* Setup PWM CNV Signal */
	printk("Setting CNV Signal\n");
	ret = pwm_set_cycles(cfg->cnv_pwm_spec.dev, cfg->cnv_pwm_spec.channel,cfg->sample_period, CNV_HIGH_TIME, cfg->cnv_pwm_spec.flags);
	if (ret < 0){
		printk("Error %d: failed to set convert signal\n", ret);
		return -1;
	}

	/* Enable Timer Interrupts */
	 LL_TIM_EnableIT_UPDATE(pwm_cfg->timer);

	return 0;
	
}

/* Probably need to add more here to ensure everything is properly reset */
static int analog_ad4002_stop_read(const struct device *dev){
	const struct ad4002_config *cfg = dev->config;
	pwm_set_cycles(cfg->cnv_pwm_spec.dev, cfg->cnv_pwm_spec.channel,cfg->sample_period, 0, cfg->cnv_pwm_spec.flags);
}

/* Internal Function to Start to Read Data following CNV interrupt */
static void analog_ad4002_read(const struct device *dev){

	int ret;
	struct ad4002_data *dev_data = dev->data;
	dev_data->N_samples = dev_data->N_samples - 1; // Decrement N_samples until it reaches 0
	const struct ad4002_config *cfg = dev->config;

	/* Stop PWM Convert Signal */
	if (dev_data->N_samples < 1){
		pwm_set_cycles(cfg->cnv_pwm_spec.dev, cfg->cnv_pwm_spec.channel,cfg->sample_period, 0, cfg->cnv_pwm_spec.flags);
		const struct device *pwm_device = cfg->cnv_pwm_spec.dev;
		const struct pwm_stm32_config *pwm_cfg = pwm_device->config;
		LL_TIM_DisableIT_UPDATE(pwm_cfg->timer);
	}
	return;
}

static int ad4002_init(const struct device *dev){
	
	const struct ad4002_config *cfg = dev->config;
	//startTime = k_uptime_get();
	
	SPI_TypeDef* spi_block = cfg->spi_block;


	/* DMA Initialization */
	struct stream *dma_stream = &(cfg->dma);
	
	DMA_Channel_TypeDef* dma_block = dma_stream->reg + 1; 
	dma_block = dma_block + dma_stream->channel -1;

	dma_block->CCR |= ((dma_stream->cfg.cyclic << DMA_CCR_CIRC_Pos) |
					(dma_stream->cfg.source_data_size << DMA_CCR_PSIZE_Pos) |
					(dma_stream->cfg.dest_data_size << DMA_CCR_MSIZE_Pos) |
					(dma_stream->src_addr_increment << DMA_CCR_PINC_Pos) |
					(dma_stream->dst_addr_increment << DMA_CCR_MINC_Pos) |
					(dma_stream->cfg.channel_priority << DMA_CCR_PL_Pos));

	dma_block->CNDTR = dma_stream->cfg.source_burst_length;
	dma_block->CPAR = &spi_block->DR;

	dma_block->CCR |= DMA_CCR_EN;

	/* DMA Clock configuration, might not be necessary since DMA driver should handle */
	if (clock_control_on(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
			     (clock_control_subsys_t) &dma_stream->pclken) != 0) {
		return -EIO;
	}

	/* Initialize SPI bus (Low Level) */
	WRITE_REG(spi_block->CR1, (SPI_CR1_CPHA | SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI)); // Basic Config
	WRITE_REG(spi_block->CR2, (SPI_CR2_RXDMAEN | (0xF << SPI_CR2_DS_Pos))); // SPI DMA Enable

	/* Configure and Enable IRQ in NVIC */
	cfg->irq_config_func(dev);

	return 0;
}

/** Fastest Possible Interrupt handler for SPI reads following Timer Update
 * The function handle will directly be placed in the MCU vector table. 
 * No data passing is permitted
*/
ISR_DIRECT_DECLARE(pwm_irq_handler_direct)
{
	// Takes about 12 cycles to write the SPI data register and clear the Timer Update Flag
	uint32_t* SPI_DR = 0x4001300C;
	WRITE_REG((*SPI_DR), 0xFFFF);
	uint32_t* timer_SR = 0x40012C10;
	CLEAR_BIT((*timer_SR), 0x1);

	ISR_DIRECT_PM();
	return 1;
}


/* Timer Style Low Latency Interrupt Handler */
void pwm_irq_handler(const struct device *dev)
{
	/* Obtain timer object from PWM device configuration structure */
	//const struct ad4002_config *cfg = dev->config;
	//const struct device *pwm_device = cfg->cnv_pwm_spec.dev;
	//struct ad4002_data *dev_data = dev->data;
	//const struct pwm_stm32_config *pwm_cfg = pwm_device->config;
	//TIM_TypeDef *timer = pwm_cfg->timer;

	/* Clear Flag */
	
	//LL_TIM_ClearFlag_UPDATE(timer);

	/** SPI Read Low-Level. Data will be written to TX buffer to initiate full-duplex transmission.
	* DMA will take care of actual transfer of data from buffer to memory. Can add extra instructions here
	* to achieve appropriate CNV - SCK timing. 
	*/
	//WRITE_REG(cfg->spi_block->DR, 0xFFFF);

	/* For debugging purposes only */
	// if (dev_data->N_samples % 100 == 0){
	// 	printk("Samples Remaining: %d\n", dev_data->N_samples);
	// }


	// Initial stack push takes about 6 cycles (5 registers + LR)

	// Takes about 12 cycles
	uint32_t* SPI_DR = 0x4001300C;
	WRITE_REG((*SPI_DR), 0xFFFF);
	uint32_t* timer_SR = 0x40012C10;
	CLEAR_BIT((*timer_SR), 0x1);


	/* Update remaining sample count. Takes 4 instructions and 7 cycles */
	struct ad4002_data *dev_data = dev->data; 
	dev_data->N_samples = dev_data->N_samples - 1; 

	/* Stop PWM Convert Signal when sample count reaches 0. Should take just a few clock cycles if no branch occurs */
	if (dev_data->N_samples < 1){
		const struct ad4002_config *cfg = dev->config;
		const struct device *pwm_device = cfg->cnv_pwm_spec.dev;
		const struct pwm_stm32_config *pwm_cfg = pwm_device->config;
		LL_TIM_DisableIT_UPDATE(pwm_cfg->timer);
		pwm_set_cycles(cfg->cnv_pwm_spec.dev, cfg->cnv_pwm_spec.channel,cfg->sample_period, 0, cfg->cnv_pwm_spec.flags);
	}
	return;

}																			

static const struct ad4002_driver_api ad4002_api = {
	.read = analog_ad4002_read,
	.continuous_read = analog_ad4002_continuous_read,
	.stop_read = analog_ad4002_stop_read,
};

#define IRQ_CONFIG_FUNC(index)											\
	static void pwm_irq_config_##index(const struct device *dev)     	\
    {                                           						\
		printk("IRQ_Connecting\n");                        				\
        IRQ_DIRECT_CONNECT(DT_IRQ_BY_NAME(PWM_PARENT_TIMER(index), up, irq),	\
			    DT_IRQ_BY_NAME(PWM_PARENT_TIMER(index), up, priority),  \
			    pwm_irq_handler_direct,				  						\
			    IRQ_ZERO_LATENCY);							            \
		irq_enable(DT_IRQ_BY_NAME(PWM_PARENT_TIMER(index), up, irq));	\
    };  

#define AD4002_DEVICE_INIT(index) 										\
																		\
	IRQ_CONFIG_FUNC(index)  											\
																		\
	static const struct ad4002_config ad4002_cfg_##index = { 			\
		.spi_block = DT_REG_ADDR(SPI_NODE(index)), 						\
		.cnv_pwm_spec = PWM_DT_SPEC_INST_GET(index),					\
		.sample_period = DT_PROP(AD4002_NODE(index), sample_period), 	\
		.irq_config_func = pwm_irq_config_##index,						\
		SPI_DMA_CHANNEL(SPI_NODE(index))								\
	}; 																	\
																		\
	static struct ad4002_data ad4002_dev_data_##index = {				\
		.N_samples = 1024,												\
	};																	\
																		\
DEVICE_DT_INST_DEFINE(index, &ad4002_init, NULL, 						\
	&ad4002_dev_data_##index, &ad4002_cfg_##index, POST_KERNEL,      						\
	CONFIG_AD4002_SPI_INIT_PRIORITY, &ad4002_api);

// Initialize each active device
DT_INST_FOREACH_STATUS_OKAY(AD4002_DEVICE_INIT)