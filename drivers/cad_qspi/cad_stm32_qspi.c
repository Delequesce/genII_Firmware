#define DT_DRV_COMPAT cad_stm32_qspi

#include <zephyr/kernel.h>
#include <zephyr/toolchain.h>
#include <zephyr/arch/common/ffs.h>
#include <zephyr/sys/util.h>
#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <string.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/dma/dma_stm32.h>
#include <stm32_ll_dma.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <app/drivers/cad_qspi.h>
#include <zephyr/logging/log.h>
#include <stm32wb55xx.h>

LOG_MODULE_REGISTER(cad_stm32_qspi);

static void qspi_dma_callback(const struct device *dev, void *arg,
			 uint32_t channel, int status);

/* Necessary structures and defines */

#define STM32_QSPI_NODE(index) DT_DRV_INST(index)
//PINCTRL_DT_DEFINE(STM32_QSPI_NODE);
#define STM32_QSPI_FIFO_THRESHOLD			16
#define STM32_QSPI_SAMPLE_SHIFTING			1             
#define STM32_QSPI_PRESCALER				0      


#define STM32_QSPI_USE_DMA                  DT_NODE_HAS_PROP(DT_DRV_INST(0), dmas)
#define DMA_CHANNEL_CONFIG(node, dir)	    DT_DMAS_CELL_BY_NAME(node, dir, channel_config)
#define QSPI_DMA_CHANNEL_INIT(node, dir)				\
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
		.source_data_size = STM32_DMA_CONFIG_PERIPHERAL_DATA_SIZE( \
					DMA_CHANNEL_CONFIG(node, dir)), \
		.dest_data_size = STM32_DMA_CONFIG_MEMORY_DATA_SIZE(    \
					DMA_CHANNEL_CONFIG(node, dir)), \
		.channel_priority = STM32_DMA_CONFIG_PRIORITY(		\
					DMA_CHANNEL_CONFIG(node, dir)), \
		.cyclic = ((DMA_CHANNEL_CONFIG(node, dir) >> 5) & 0x1), \
		.source_burst_length = 1, \
		.dest_burst_length = 1, \
	},								\
	.src_addr_increment = STM32_DMA_CONFIG_PERIPHERAL_ADDR_INC(DMA_CHANNEL_CONFIG(node, dir)), \
	.dst_addr_increment = STM32_DMA_CONFIG_MEMORY_ADDR_INC(DMA_CHANNEL_CONFIG(node, dir)),

#define QSPI_DMA_CHANNEL(node, dir)					\
	.dma = {							\
		COND_CODE_1(DT_DMAS_HAS_NAME(node, dir),		\
			(QSPI_DMA_CHANNEL_INIT(node, dir)),		\
			(NULL))						\
		},

struct cad_qspi_config {
	QUADSPI_TypeDef *regs;
	struct stm32_pclken pclken;
	const struct pinctrl_dev_config *pcfg;
};

struct stream {
	DMA_TypeDef *reg;
	const struct device *dev;
	uint32_t channel;
	struct dma_config cfg;
	struct stm32_pclken pclken;
	bool src_addr_increment;
	bool dst_addr_increment;
};

#if STM32_QSPI_USE_DMA
static const uint32_t table_m_size[] = {
	LL_DMA_MDATAALIGN_BYTE,
	LL_DMA_MDATAALIGN_HALFWORD,
	LL_DMA_MDATAALIGN_WORD,
};

static const uint32_t table_p_size[] = {
	LL_DMA_PDATAALIGN_BYTE,
	LL_DMA_PDATAALIGN_HALFWORD,
	LL_DMA_PDATAALIGN_WORD,
};


/* Lookup table to set dma priority from the DTS */
static const uint32_t table_priority[] = {
	DMA_PRIORITY_LOW,
	DMA_PRIORITY_MEDIUM,
	DMA_PRIORITY_HIGH,
	DMA_PRIORITY_VERY_HIGH,
};
#endif 

struct cad_qspi_data {
	QUADSPI_TypeDef *Instance;
	uint32_t FifoThreshold;
	uint32_t SampleShifting;
	uint32_t Prescaler;
	int cmd_status;
	uint16_t qspi_read_cmd;
    struct stream dma;
};

/* Exposed Functions */

static int cad_qspi_read(const struct device *dev, uint32_t *data, uint16_t size){

	struct cad_qspi_data *dev_data = dev->data;
	QUADSPI_TypeDef* qspi_block = (dev_data->Instance);

	qspi_block->DLR = (size-1);

	/* This should configure and start the read operation */
	qspi_block->CCR |= ((1 << QUADSPI_CCR_FMODE_Pos) |
						(2 << QUADSPI_CCR_DMODE_Pos) |
						(2 << QUADSPI_CCR_DCYC_Pos)  |
						(1 << QUADSPI_CCR_IMODE_Pos));


	qspi_block->CR |= (QUADSPI_CR_EN); 

	// Send Arbitrary command
	qspi_block->CCR |= (0x32 << QUADSPI_CCR_INSTRUCTION_Pos);

	/* Wait for command to complete, then disable the peripheral*/
	while (QUADSPI->SR & QUADSPI_SR_BUSY) {};
	qspi_block->CR &= ~(QUADSPI_CR_EN); 
    
	#if STM32_QSPI_USE_DMA

	#else

	/* Check TCF and read data */
	while (((qspi_block->SR) & QUADSPI_SR_TCF) == 0){};
	(*data) =  qspi_block->DR;
	qspi_block->FCR = QUADSPI_FCR_CTCF;

	#endif

	dev_data->cmd_status = 1; 

    return dev_data->cmd_status;
}
/* End Exposed Functions */


/* Internal Functions*/
static int cad_qspi_init(const struct device *dev)
{
	uint32_t ahb_clock_freq;
	const struct cad_qspi_config *dev_cfg = dev->config;
	struct cad_qspi_data *dev_data = dev->data;
	int ret;

	/* Signals configuration */
	ret = pinctrl_apply_state(dev_cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("QSPI pinctrl setup failed (%d)", ret);
		return ret;
	}

	/* Configure and enable DMA */
    #if STM32_QSPI_USE_DMA

	struct stream *dma_stream = &(dev_data->dma);
	DMA_Channel_TypeDef* dma_block = (&(dma_stream->reg) + DMA_GET_CHANNEL_OFFSET(dma_stream->channel));

	dma_block->CCR |= ((dma_stream->cfg.cyclic << DMA_CCR_CIRC_Pos) |
					(dma_stream->cfg.source_data_size << DMA_CCR_PSIZE_Pos) |
					(dma_stream->cfg.dest_data_size << DMA_CCR_MSIZE_Pos) |
					(dma_stream->src_addr_increment << DMA_CCR_PINC_Pos) |
					(dma_stream->dst_addr_increment << DMA_CCR_MINC_Pos) |
					(dma_stream->cfg.channel_priority << DMA_CCR_PL_Pos));

	dma_block->CCR |= DMA_CCR_EN;


	/* DMA Clock configuration */
	if (clock_control_on(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
			     (clock_control_subsys_t) &dma_stream->pclken) != 0) {
		LOG_DBG("Could not enable DMA clock");
		return -EIO;
	}

    #endif

	/* QSPI Clock configuration */
	if (clock_control_on(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
			     (clock_control_subsys_t) &dev_cfg->pclken) != 0) {
		LOG_DBG("Could not enable QSPI clock");
		return -EIO;
	}

	if (clock_control_get_rate(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
			(clock_control_subsys_t) &dev_cfg->pclken,
			&ahb_clock_freq) < 0) {
		LOG_DBG("Failed to get AHB clock frequency");
		return -EIO;
	}

	// Direct Register Interactions
	QUADSPI_TypeDef* qspi_block = (dev_cfg->regs);

	// Specify flash size (might be necessary), completely arbitrary
	qspi_block->DCR |=  ( 25 << QUADSPI_DCR_FSIZE_Pos );

	/* Write to control register */
	uint32_t cr_data = (((dev_data->FifoThreshold -1) << QUADSPI_CR_FTHRES_Pos) | ((dev_data->SampleShifting) << QUADSPI_CR_SSHIFT_Pos) | ((dev_data->Prescaler) << QUADSPI_CR_PRESCALER_Pos));
	qspi_block->CR |= cr_data;

	return 0;
}

/* End Internal Functions*/

/* API Exposure and Routing*/
static const struct cad_qspi_driver_api stm32_qspi_driver_api = {
	.read = cad_qspi_read,
};

#define QSPI_DEVICE_INIT(index)                                                 \
static struct cad_qspi_data cad_qspi_dev_data_##index = {                       \
	.Instance = (QUADSPI_TypeDef *)DT_REG_ADDR(STM32_QSPI_NODE(index)),     \
	.FifoThreshold = STM32_QSPI_FIFO_THRESHOLD,                     \
	.SampleShifting = STM32_QSPI_SAMPLE_SHIFTING,                    \
	.Prescaler = STM32_QSPI_PRESCALER,								\
    QSPI_DMA_CHANNEL(STM32_QSPI_NODE(index), rx)                            \
};                                                                          \
PINCTRL_DT_INST_DEFINE(index);	                                            \
static const struct cad_qspi_config cad_qspi_cfg_##index = {                \
	.regs = (QUADSPI_TypeDef *)DT_REG_ADDR(STM32_QSPI_NODE(index)),         \
	.pclken = {                                                             \
		.enr = DT_CLOCKS_CELL(STM32_QSPI_NODE(index), bits),                \
		.bus = DT_CLOCKS_CELL(STM32_QSPI_NODE(index), bus),                 \
	},                                                                      \
	.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),                          \
};                                                                          \
                                                                            \
	                                                                        \
DEVICE_DT_INST_DEFINE(index, &cad_qspi_init, NULL,                          \
&cad_qspi_dev_data_##index, &cad_qspi_cfg_##index,                          \
POST_KERNEL, CONFIG_QSPI_INIT_PRIORITY, &stm32_qspi_driver_api);

// Initialize each active device
DT_INST_FOREACH_STATUS_OKAY(QSPI_DEVICE_INIT)

/* End API Exposure and Routing */