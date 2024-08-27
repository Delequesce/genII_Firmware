#define DT_DRV_COMPAT analog_ad4002

#include <zephyr/device.h>
#include <zephyr/sys/util_macro.h>
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
#define PWM_NODE(index)  DT_PWMS_CTLR_BY_IDX(AD4002_NODE(index), 0) // Gets the PWM Node that serves as the convert signal interrupt timer
#define PWM_PARENT_TIMER(index) DT_PARENT(PWM_NODE(index)) // Gets the parent timer of the aforementioned PWM node for interrupt vectoring
#define SPI_NODE(index) DT_BUS(AD4002_NODE(index))

/* SPI and PWM Params*/
#define CNV_HIGH_TIME 		21 // 21 Clock cycles is about 330 nsec at 64 MHz
#define CNV_HEADSUP_TIME	16 // 10 Clock cycles is about 160 nsec at 64 MHz
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
		.channel_direction = ((DMA_CHANNEL_CONFIG(node, dir) >> 6) & 0x3), \
	},								\
	.periph_addr_increment = STM32_DMA_CONFIG_PERIPHERAL_ADDR_INC(DMA_CHANNEL_CONFIG(node, dir)), \
	.mem_addr_increment = STM32_DMA_CONFIG_MEMORY_ADDR_INC(DMA_CHANNEL_CONFIG(node, dir)),

#define SPI_DMA_CHANNEL(node)							\
	.dma_rx = {											\
		COND_CODE_1(DT_DMAS_HAS_NAME(node, rx),			\
			(SPI_DMA_CHANNEL_INIT(node, rx)),			\
			(NULL))										\
		},												\
	.dma_tx = {											\
		COND_CODE_1(DT_DMAS_HAS_NAME(node, tx),			\
			(SPI_DMA_CHANNEL_INIT(node, tx)),			\
			(NULL))										\
		},

struct stream {
	DMA_TypeDef *reg;
	const struct device *dev;
	uint32_t channel;
	struct dma_config cfg;
	struct stm32_pclken pclken;
	bool periph_addr_increment;
	bool mem_addr_increment;
};

uint64_t startTime;
uint16_t numSamples;
uint32_t ll_channel;

// Manual Read Data structures
static uint16_t rx_data[1024];
uint32_t count; 


/* Read only configuration data set at build time*/
struct ad4002_config{
	SPI_TypeDef* spi_block;
	struct pwm_dt_spec cnv_pwm_spec;
	uint8_t sample_period; 
	void (*dma_irq_config_func)(const struct device *dev);
	struct stream dma_rx; 
	struct stream dma_tx;
	bool master;
	uint8_t instance;
	uint16_t tx_buffer;
};

/* Mutable data, stored in RAM */
struct ad4002_data{
	uint32_t N_samples;
	ad4002_irq_callback_user_data_t user_cb;
};

/** PWM configuration. */
struct pwm_stm32_config {
	TIM_TypeDef *timer;
	uint32_t prescaler;
	uint32_t countermode;
	struct stm32_pclken pclken;
	const struct pinctrl_dev_config *pcfg;
};

/** Timer Channel to LL mapping. */
static const uint32_t ch2ll[6] = {
	LL_TIM_CHANNEL_CH1, LL_TIM_CHANNEL_CH2,
	LL_TIM_CHANNEL_CH3, LL_TIM_CHANNEL_CH4,
	LL_TIM_CHANNEL_CH5, LL_TIM_CHANNEL_CH6
};

static int analog_ad4002_init_read(const struct device *dev_master, const struct device *dev_slave, uint16_t* rx_buffer_1, uint16_t* rx_buffer_2, const uint32_t N_samples)
{
	int ret;
	count = 0; 

	/* Sets up SPI and DMA for each ADC */
	spi_dma_setup(dev_master, rx_buffer_1, N_samples, true);
	spi_dma_setup(dev_slave, rx_buffer_2, N_samples, false);
	
	/* Setup PWM CNV Signal and heads-up timer */
	const struct ad4002_config *cfg = dev_master->config;
	const struct device *pwm_device = cfg->cnv_pwm_spec.dev;
	const struct pwm_stm32_config *pwm_cfg = pwm_device->config;

	/* Setup the CNV PWM Signal */
	LL_TIM_EnableCounter(pwm_cfg->timer);
	ll_channel = ch2ll[cfg->cnv_pwm_spec.channel - 1u];
	LL_TIM_OC_SetPolarity(pwm_cfg->timer, ll_channel, LL_TIM_OCPOLARITY_HIGH);
	LL_TIM_OC_SetCompareCH1(pwm_cfg->timer, CNV_HIGH_TIME); // For CNV Timer, heads up is already set earlier
	LL_TIM_EnableARRPreload(pwm_cfg->timer);
	LL_TIM_OC_EnablePreload(pwm_cfg->timer, ll_channel);
	LL_TIM_SetAutoReload(pwm_cfg->timer, cfg->sample_period);

	return 0;
	
}

static int spi_dma_setup(const struct device *dev, uint16_t* rx_buffer, const uint32_t N_samples, const bool tx_rx){
	const struct ad4002_config *cfg = dev->config;
	struct ad4002_data *dev_data = dev->data;
	SPI_TypeDef* spi_block = cfg->spi_block;

	/* Set DMA memory location */
	struct stream *dma_stream_rx = &(cfg->dma_rx);
	DMA_Channel_TypeDef* dma_block_rx = dma_stream_rx->reg + 1; 
	dma_block_rx = dma_block_rx + dma_stream_rx->channel -1;
	DMA_Channel_TypeDef* dma_block_tx;

	/* DMA Address and Data Length Config */
	WRITE_REG(dma_block_rx->CMAR, rx_buffer);
	WRITE_REG(dma_block_rx->CPAR, &spi_block->DR);
	WRITE_REG(dma_block_rx->CNDTR, N_samples);
	if(tx_rx){
		struct stream *dma_stream_tx = &(cfg->dma_tx);
		dma_block_tx = dma_stream_tx->reg + 1; 
		dma_block_tx = dma_block_tx + dma_stream_tx->channel -1;
		/* DMA Address and Data Length Config */
		WRITE_REG(dma_block_tx->CMAR, &cfg->tx_buffer);
		WRITE_REG(dma_block_tx->CPAR, &spi_block->DR);
		WRITE_REG(dma_block_tx->CNDTR, N_samples);

		/* Set up DMA callback */
		// To Do...
	}

	/* Enable SPI DMA RX Request, then enable SPI, then finally enable DMA, */
	SET_BIT(spi_block->CR2, (SPI_CR2_RXDMAEN));

	SET_BIT(spi_block->CR1, SPI_CR1_SPE);

	SET_BIT(dma_block_rx->CCR, DMA_CCR_EN);
	if(tx_rx){
		SET_BIT(dma_block_tx->CCR, DMA_CCR_EN);
	}

}

/* Probably need to add more here to ensure everything is properly reset */
static int analog_ad4002_start_read(const struct device *dev, const uint32_t N_samples){
	const struct ad4002_config *cfg = dev->config;
	const struct pwm_stm32_config *pwm_cfg = cfg->cnv_pwm_spec.dev->config;

	/* Reset TX DMA CNDTR */
	struct stream *dma_stream_tx = &(cfg->dma_tx);
	DMA_Channel_TypeDef* dma_block_tx = dma_stream_tx->reg + 1; 
	dma_block_tx = dma_block_tx + dma_stream_tx->channel -1;
	CLEAR_BIT(dma_block_tx->CCR, DMA_CCR_EN);
	WRITE_REG(dma_block_tx->CNDTR, N_samples);
	SET_BIT(dma_block_tx->CCR, DMA_CCR_EN);

	/* Enable the Timer 1 Output compares */
	ll_channel = ch2ll[cfg->cnv_pwm_spec.channel - 1u];

	LL_TIM_OC_SetMode(pwm_cfg->timer, ll_channel, LL_TIM_OCMODE_PWM1);
	GPIO_TypeDef *pa = GPIOA;
	MODIFY_REG(pa->MODER, GPIO_MODER_MODE8_0, GPIO_MODER_MODE8_1);
	LL_TIM_CC_EnableChannel(pwm_cfg->timer, (ll_channel | LL_TIM_CHANNEL_CH2));
	LL_TIM_GenerateEvent_UPDATE(pwm_cfg->timer);

	return 0; 
}

static int analog_ad4002_irq_callback_set(const struct device *dev, ad4002_irq_callback_user_data_t cb)
{
	struct ad4002_data *dev_data = dev->data;
	struct ad4002_config *cfg = dev->config;
	dev_data->user_cb = cb;

	return 0;
}

static int ad4002_init(const struct device *dev){
	
	const struct ad4002_config *cfg = dev->config;

	/* DMA Clock configuration, might not be necessary since DMA driver should handle */
	struct stream *dma_stream_rx = &(cfg->dma_rx);
	if (clock_control_on(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
			     (clock_control_subsys_t) &dma_stream_rx->pclken) != 0) {
		return -EIO;
	}
	struct stream *dma_stream_tx = &(cfg->dma_tx);

	// Enable DMA Mux Clock
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);

	SPI_TypeDef* spi_block = cfg->spi_block;

	/* Timer Setup */
	if (cfg->master){
		const struct device *pwm_device = cfg->cnv_pwm_spec.dev;
		const struct pwm_stm32_config *pwm_cfg = pwm_device->config;
		TIM_TypeDef* tim_block = pwm_cfg->timer;
		//WRITE_REG(tim_block->CCR2, 64-CNV_HEADSUP_TIME); // Set Duty cycle for heads-up timer
		WRITE_REG(tim_block->CCR2, CNV_HEADSUP_TIME); // Set Duty cycle for heads-up timer
		WRITE_REG(tim_block->DIER, (TIM_DIER_CC2DE)); // Trigger a DMA request on channel 2 capture compare

	}

	/* DMA Initialization */
	DMA_TypeDef* dma_base_block_rx = dma_stream_rx->reg;
	DMA_Channel_TypeDef* dma_block_rx = dma_stream_rx->reg + 1; 
	dma_block_rx = dma_block_rx + dma_stream_rx->channel -1;

	LL_DMA_SetPeriphRequest(dma_base_block_rx, dma_stream_rx->channel, dma_stream_rx->cfg.dma_slot); // Sets up DMA Mux

	dma_block_rx->CCR |= ((dma_stream_rx->cfg.cyclic << DMA_CCR_CIRC_Pos) |
				(dma_stream_rx->cfg.source_data_size << DMA_CCR_PSIZE_Pos) |
				(dma_stream_rx->cfg.dest_data_size << DMA_CCR_MSIZE_Pos) |
				(dma_stream_rx->periph_addr_increment << DMA_CCR_PINC_Pos) |
				(dma_stream_rx->mem_addr_increment << DMA_CCR_MINC_Pos) |
				(dma_stream_rx->cfg.channel_priority << DMA_CCR_PL_Pos));

	if(cfg->master){

		DMA_TypeDef* dma_base_block_tx = dma_stream_tx->reg;
		DMA_Channel_TypeDef* dma_block_tx = dma_stream_tx->reg + 1; 
		dma_block_tx = dma_block_tx + dma_stream_tx->channel -1;

		LL_DMA_SetPeriphRequest(dma_base_block_tx, dma_stream_tx->channel, dma_stream_tx->cfg.dma_slot); // Sets up DMA Mux

		dma_block_tx->CCR |= ((dma_stream_tx->cfg.cyclic << DMA_CCR_CIRC_Pos) |
					(dma_stream_tx->cfg.source_data_size << DMA_CCR_MSIZE_Pos) |
					(dma_stream_tx->cfg.dest_data_size << DMA_CCR_PSIZE_Pos) |
					(dma_stream_tx->periph_addr_increment << DMA_CCR_PINC_Pos) |
					(dma_stream_tx->mem_addr_increment << DMA_CCR_MINC_Pos) |
					(dma_stream_tx->cfg.channel_priority << DMA_CCR_PL_Pos) |
					(dma_stream_tx->cfg.channel_direction << DMA_CCR_DIR_Pos));

		cfg->dma_irq_config_func(dev);
		SET_BIT(dma_block_tx->CCR, DMA_CCR_TCIE); // Enables interrupt request on transfer complete
	}

	/* Initialize SPI bus (Low Level) */
	if (cfg->master){
		WRITE_REG(spi_block->CR1, (SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI)); // Basic Master Config
		SET_BIT(spi_block->CR2, SPI_CR2_DS_3); // 16b 
	}
	else{
		WRITE_REG(spi_block->CR1, (SPI_CR1_SSM)); // Basic Slave Config
		SET_BIT(spi_block->CR2, SPI_CR2_DS_3); // 16b 
	}

	return 0;
}


static const struct ad4002_driver_api ad4002_api = {
	.init_read = analog_ad4002_init_read,
	.start_read = analog_ad4002_start_read,
	.irq_callback_set = analog_ad4002_irq_callback_set,
};

/* This is the callback function for when the request number of samples have been read and transferred by DMA1, CH2. Should not be direct declare because we need  */
void dma_irq_tcie(const struct device *dev)
{
	/* Disable Timer 1 Output compare and wake up */
	const struct ad4002_config *cfg = dev->config;
	const struct device *pwm_device = cfg->cnv_pwm_spec.dev;
	const struct pwm_stm32_config *pwm_cfg = pwm_device->config;

	ll_channel = ch2ll[cfg->cnv_pwm_spec.channel - 1u];
	//LL_TIM_OC_SetMode(pwm_cfg->timer, ll_channel, LL_TIM_OCMODE_FORCED_INACTIVE);
	LL_TIM_CC_DisableChannel(pwm_cfg->timer, (ll_channel|LL_TIM_CHANNEL_CH2));
	// Set GPIO pin to Output (Ch1 PA8)
	GPIO_TypeDef *pa = GPIOA;
	MODIFY_REG(pa->MODER, GPIO_MODER_MODE8_1, GPIO_MODER_MODE8_0);

	/* Clear Flags */
	uint32_t* SPI_DMA_1_IFCR = DMA1_BASE + 0x04;
	WRITE_REG((*SPI_DMA_1_IFCR), 0xFFFF);

	/* Call callback function */
	struct ad4002_data *dev_data = dev->data;
	dev_data->user_cb(dev);
}


// Below, 0xC is the NVIC interrupt position for DMA1_CH2
#define IRQ_DMA_TCIE_CONFIG_MASTER(index)											\
static void setup_dma_tcie_callback_##index(const struct device *dev){				\
			printk("DMA MASTER Interrupt enabled\n");                   			\
			IRQ_CONNECT(0xD, 0, dma_irq_tcie, DEVICE_DT_INST_GET(index), 0);		\
			irq_enable(0xD);														\
}

#define IRQ_DMA_TCIE_CONFIG_SLAVE(index)											\
static void setup_dma_tcie_callback_##index(const struct device *dev){				\
			printk("DMA SLAVE Interrupt enabled\n");                   				\
}
																		
#define AD4002_NODE(index) 	DT_DRV_INST(index) 
#define SPI_MASTER(index) 	DT_PROP(AD4002_NODE(index), spi_master)
#define PWM_RETURN(index) 	COND_CODE_1(SPI_MASTER(index), (PWM_DT_SPEC_INST_GET(index)), (NULL))
#define IRQ_DMA_TCIE_CONFIG(index)  COND_CODE_1(SPI_MASTER(index), (IRQ_DMA_TCIE_CONFIG_MASTER(index)), (IRQ_DMA_TCIE_CONFIG_SLAVE(index)));

#define AD4002_DEVICE_INIT(index) 										\
																		\
	IRQ_DMA_TCIE_CONFIG(index);											\
																		\
	static const struct ad4002_config ad4002_cfg_##index = { 			\
		.spi_block = DT_REG_ADDR(SPI_NODE(index)), 						\
		.cnv_pwm_spec = PWM_RETURN(index),								\
		.sample_period = DT_INST_PROP_OR(index, sample_period, 0),		\
		.dma_irq_config_func = setup_dma_tcie_callback_##index,			\
		SPI_DMA_CHANNEL(SPI_NODE(index))						        \
		.master = SPI_MASTER(index),									\
		.instance = index,												\
		.tx_buffer = 0xFFFF,											\
	}; 																	\
																		\
	static struct ad4002_data ad4002_dev_data_##index = {				\
		.N_samples = 1024,												\
	};																	\
																		\
DEVICE_DT_INST_DEFINE(index, &ad4002_init, NULL, 						\
	&ad4002_dev_data_##index, &ad4002_cfg_##index, POST_KERNEL,      	\
	CONFIG_AD4002_SPI_INIT_PRIORITY, &ad4002_api);						

// Initialize each active device
DT_INST_FOREACH_STATUS_OKAY(AD4002_DEVICE_INIT)