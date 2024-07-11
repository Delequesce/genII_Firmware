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
#define SPI_OPER(index) 	(SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(SPI_FRAME_BITS))
#define SPI_FRAME_BITS 		16
#define CNV_HIGH_TIME 		21 // 21 Clock cycles is about 330 nsec at 64 MHz
#define CNV_HEADSUP_TIME	8 // 10 Clock cycles is about 160 nsec at 64 MHz
#define RX_BUFFER_LENGTH    2

// Comment out to switch between manual and DMA read modes
#define USE_DMA
#define USE_DMA_INTERRUPT

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

// Manual Read Data structures
static uint16_t rx_data[1024];
uint32_t count; 

/* Read only configuration data set at build time*/
struct ad4002_config{
	SPI_TypeDef* spi_block;
	struct pwm_dt_spec cnv_pwm_spec;
	uint8_t sample_period; 
	void (*irq_config_func)(const struct device *dev);
	void (*data_irq_func)(const struct device *dev);
	struct stream dma; 
	bool master;
	uint8_t instance;
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

static int analog_ad4002_continuous_read(const struct device *dev, uint16_t* rx_buffer, const uint32_t N_samples)
{
	int ret;
	count = 0; 
	const struct ad4002_config *cfg = dev->config;
	struct ad4002_data *dev_data = dev->data;
	SPI_TypeDef* spi_block = cfg->spi_block;
	dev_data->N_samples = N_samples;

	/* Set DMA memory location */
	#ifdef USE_DMA
	struct stream *dma_stream = &(cfg->dma);
	DMA_Channel_TypeDef* dma_block = dma_stream->reg + 1; 
	dma_block = dma_block + dma_stream->channel -1;
	/* DMA Address and Data Length Config */
	WRITE_REG(dma_block->CMAR, rx_buffer);
	WRITE_REG(dma_block->CPAR, &spi_block->DR);
	WRITE_REG(dma_block->CNDTR, dma_stream->cfg.source_burst_length);
	#endif

	/* Enable SPI DMA RX Request, then enable SPI, then finally enable DMA, */
	#ifdef USE_DMA
	SET_BIT(spi_block->CR2, (SPI_CR2_RXDMAEN));
	#endif

	SET_BIT(spi_block->CR1, SPI_CR1_SPE);

	#ifdef USE_DMA
	SET_BIT(dma_block->CCR, DMA_CCR_EN);
	#endif
	if (cfg->master){ 
		
		/* Setup PWM CNV Signal and heads-up timer */
		//printk("Setting PWM CNV Signal\n");
		const struct device *pwm_device = cfg->cnv_pwm_spec.dev;
		const struct pwm_stm32_config *pwm_cfg = pwm_device->config;
		pwm_set_cycles(cfg->cnv_pwm_spec.dev, cfg->cnv_pwm_spec.channel,cfg->sample_period, CNV_HIGH_TIME, cfg->cnv_pwm_spec.flags);
		
		/* Enable Timer Interrupts */
		SET_BIT(pwm_cfg->timer->DIER, TIM_DIER_CC2IE);
	}
	return 0;
	
}

/* Probably need to add more here to ensure everything is properly reset */
static int analog_ad4002_stop_read(const struct device *dev){
	const struct ad4002_config *cfg = dev->config;
	printk("Stopping Read");
	count = 0;
	if(cfg->master){
		pwm_set_cycles(cfg->cnv_pwm_spec.dev, cfg->cnv_pwm_spec.channel,cfg->sample_period, 0, cfg->cnv_pwm_spec.flags);
	}
	return rx_data; 
}

/* Internal Function to Start to Read Data following CNV interrupt */
static void analog_ad4002_read(const struct device *dev){

	return;
}

static int ad4002_init(const struct device *dev){
	
	const struct ad4002_config *cfg = dev->config;
	//startTime = k_uptime_get();

	/* DMA Clock configuration, might not be necessary since DMA driver should handle */
	struct stream *dma_stream = &(cfg->dma);
	if (clock_control_on(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
			     (clock_control_subsys_t) &dma_stream->pclken) != 0) {
		return -EIO;
	}

	// Enable DMA Mux Clock
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);
	
	SPI_TypeDef* spi_block = cfg->spi_block;

	/* Timer Setup */
	if (cfg->master){
		const struct device *pwm_device = cfg->cnv_pwm_spec.dev;
		const struct pwm_stm32_config *pwm_cfg = pwm_device->config;
		TIM_TypeDef* tim_block = pwm_cfg->timer;
		CLEAR_BIT(tim_block->CR1, TIM_CR1_CEN); // Disable Timer 1
		WRITE_REG(tim_block->CCR2, 64-CNV_HEADSUP_TIME); // Set Duty cycle for heads-up timer
		SET_BIT(tim_block->CCER, TIM_CCER_CC2E); // Enable Cpature Compare Channel 2
		WRITE_REG(tim_block->DIER, (TIM_DIER_CC2IE)); // Trigger an interrupt only on channel 2 capture compare
		SET_BIT(tim_block->CR1, TIM_CR1_CEN); // Enable Timer 1
	}

	/* DMA Initialization (LL Drivers) */
	#ifdef USE_DMA
	DMA_TypeDef* dma_base_block = dma_stream->reg;
	DMA_Channel_TypeDef* dma_block = dma_stream->reg + 1; 
	dma_block = dma_block + dma_stream->channel -1;

	LL_DMA_SetPeriphRequest(dma_base_block, dma_stream->channel, dma_stream->cfg.dma_slot); // Sets up DMA Mux
	/*LL_DMA_SetDataTransferDirection(dma_base_block, dma_stream->channel, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetChannelPriorityLevel(dma_base_block, dma_stream->channel, dma_stream->cfg.channel_priority);
	LL_DMA_SetMode(dma_base_block, dma_stream->channel, dma_stream->cfg.cyclic);
	LL_DMA_SetPeriphIncMode(dma_base_block, dma_stream->channel, dma_stream->src_addr_increment);
	LL_DMA_SetMemoryIncMode(dma_base_block, dma_stream->channel, dma_stream->dst_addr_increment);
	LL_DMA_SetPeriphSize(dma_base_block, dma_stream->channel, LL_DMA_PDATAALIGN_HALFWORD);
	LL_DMA_SetMemorySize(dma_base_block, dma_stream->channel, LL_DMA_MDATAALIGN_HALFWORD);*/

	dma_block->CCR |= ((dma_stream->cfg.cyclic << DMA_CCR_CIRC_Pos) |
				(dma_stream->cfg.source_data_size << DMA_CCR_PSIZE_Pos) |
				(dma_stream->cfg.dest_data_size << DMA_CCR_MSIZE_Pos) |
				(dma_stream->src_addr_increment << DMA_CCR_PINC_Pos) |
				(dma_stream->dst_addr_increment << DMA_CCR_MINC_Pos) |
				(dma_stream->cfg.channel_priority << DMA_CCR_PL_Pos));

	/*WRITE_REG(dma_block->CNDTR, dma_stream->cfg.source_burst_length);
	WRITE_REG(dma_block->CPAR, &spi_block->DR);*/
	#endif

	/* Initialize SPI bus (Low Level) */
	if (cfg->master){
		WRITE_REG(spi_block->CR1, (SPI_CR1_CPHA | SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI)); // Basic Master Config
	}
	else{
		WRITE_REG(spi_block->CR1, (SPI_CR1_CPHA | SPI_CR1_SSM | SPI_CR1_SSI)); // Basic Slave Config	
	}

	cfg->data_irq_func(dev);
	#ifdef USE_DMA
		#ifdef USE_DMA_INTERRUPT
		if (cfg->master){
			DMA_Channel_TypeDef* dma_block = dma_stream->reg + 1; 
			dma_block = dma_block + dma_stream->channel -1;
			MODIFY_REG(dma_block->CCR, 0, (DMA_CCR_TCIE));
		}
		#endif
	#else
		if (cfg->master){
			SET_BIT(spi_block->CR2, SPI_CR2_RXNEIE); // Set for RX interrpt with manual read
		}
	#endif 	
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
	// Takes about 12 cycles to write the SPI data register and clear the Timer Interrupt Flag
	//printk("H\n");
	uint32_t* SPI_SLAVE_CR1 = SPI2_BASE;
	CLEAR_BIT((*SPI_SLAVE_CR1), SPI_CR1_SSI);
	uint32_t* SPI_MASTER_DR = SPI1_BASE + 0xC;
	WRITE_REG((*SPI_MASTER_DR), 0xFFFF);
	uint32_t* timer_SR = TIM1_BASE + 0x10;
	CLEAR_BIT((*timer_SR), TIM_SR_CC2IF);
	ISR_DIRECT_PM();

	return 1;
}

#ifdef USE_DMA_INTERRUPT
ISR_DIRECT_DECLARE(dma_irq_tcie_direct)
{
	//printk("%x\n", READ_REG(DMA1_BASE));
	uint32_t* SPI_DMA_1_IFCR = DMA1_BASE + 0x04;
	WRITE_REG((*SPI_DMA_1_IFCR), 0xFFFF);
	//printk("Transfer Completed\n");
	count++;
	if (count > 1024){
		uint32_t* TIM1_CR = TIM1_BASE;
		CLEAR_BIT((*TIM1_CR), TIM_CR1_CEN); // Enable Timer 1
	}
	return 1;
}
#else
ISR_DIRECT_DECLARE(spi_irq_handler_direct)
{
	uint32_t* SPI_MASTER_DR = SPI1_BASE + 0xC;
	uint16_t data = READ_REG((*SPI_MASTER_DR));
	*(rx_data + count) = data;
	//printk("Data: %d\n", rx_data[count]);
	//printk("Data: %d\n", READ_REG((*SPI_MASTER_DR)));
	count++;
	if (count > 1024){
		uint32_t* TIM1_CR = TIM1_BASE;
		CLEAR_BIT((*TIM1_CR), TIM_CR1_CEN); // Enable Timer 1
	}
	return 1;
}
#endif


static const struct ad4002_driver_api ad4002_api = {
	.read = analog_ad4002_read,
	.continuous_read = analog_ad4002_continuous_read,
	.stop_read = analog_ad4002_stop_read,
};

#ifdef USE_DMA_INTERRUPT
#define IRQ_CONFIG_FUNC_DMA_TCIE(index)												\
static void setUpRXInterrupt_##index(const struct device *dev){						\
	printk("DMA INTERRUPT ENABLED\n");                        						\
	IRQ_DIRECT_CONNECT(0xC,															\
			0,  																	\
			dma_irq_tcie_direct,				  									\
			IRQ_ZERO_LATENCY);							           	 				\
	irq_enable(0xC);																\
}

#else
#define IRQ_CONFIG_FUNC_MANUAL_READ(index)											\
static void setUpRXInterrupt_##index(const struct device *dev){						\
	printk("Manual Read enabled\n");                        						\
			IRQ_DIRECT_CONNECT(DT_IRQ(SPI_NODE(index), irq),						\
					DT_IRQ(SPI_NODE(index), priority),  							\
					spi_irq_handler_direct,				  							\
					IRQ_ZERO_LATENCY);							           	 		\
			irq_enable(DT_IRQ(SPI_NODE(index), irq));								\
}
#endif

#define IRQ_CONFIG_FUNC_MASTER(index)												\
	static void pwm_irq_config_##index(const struct device *dev)     				\
    {   						                                  					\
			printk("Master IRQ Connecting\n");                        				\
			IRQ_DIRECT_CONNECT(DT_IRQ_BY_NAME(PWM_PARENT_TIMER(index), cc, irq),	\
					DT_IRQ_BY_NAME(PWM_PARENT_TIMER(index), cc, priority),  		\
					pwm_irq_handler_direct,				  							\
					IRQ_ZERO_LATENCY);							           	 		\
			irq_enable(DT_IRQ_BY_NAME(PWM_PARENT_TIMER(index), cc, irq));			\
    }; 

#define IRQ_CONFIG_FUNC_SLAVE(index)												\
	static void pwm_irq_config_##index(const struct device *dev)     				\
    {   						                                  					\
			printk("Slave IRQ Connecting\n");                        				\
    };  																		

#define AD4002_NODE(index) 	DT_DRV_INST(index) 
#define SPI_MASTER(index) 	DT_PROP(AD4002_NODE(index), spi_master)
#define PWM_RETURN(index) 	COND_CODE_1(SPI_MASTER(index), (PWM_DT_SPEC_INST_GET(index)), (NULL))
#define IRQ_DECLARE(index)  COND_CODE_1(SPI_MASTER(index), (IRQ_CONFIG_FUNC_MASTER(index)), (IRQ_CONFIG_FUNC_SLAVE(index)));
#ifdef USE_DMA_INTERRUPT
#define IRQ_DATA_FUNC(index) IRQ_CONFIG_FUNC_DMA_TCIE(index)
#else
#define IRQ_DATA_FUNC(index) IRQ_CONFIG_FUNC_MANUAL_READ(index)
#endif

#define AD4002_DEVICE_INIT(index) 										\
																		\
	IRQ_DECLARE(index);													\
	IRQ_DATA_FUNC(index);												\
																		\
	static const struct ad4002_config ad4002_cfg_##index = { 			\
		.spi_block = DT_REG_ADDR(SPI_NODE(index)), 						\
		.cnv_pwm_spec = PWM_RETURN(index),								\
		.sample_period = DT_INST_PROP_OR(index, sample_period, 0),		\
		.irq_config_func = pwm_irq_config_##index,						\
		.data_irq_func = setUpRXInterrupt_##index,						\
		SPI_DMA_CHANNEL(SPI_NODE(index))						        \
		.master = SPI_MASTER(index),									\
		.instance = index,												\
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