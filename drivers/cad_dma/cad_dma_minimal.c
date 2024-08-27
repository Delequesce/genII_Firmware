#define DT_DRV_COMPAT cad_dma_minimal

/*
 * Copyright (c) 2016 Linaro Limited.
 * Copyright (c) 2019 Song Qiang <songqiang1304521@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Common part of DMA drivers for stm32.
 * @note  Functions named with stm32_dma_* are SoCs related functions
 *        implemented in dma_stm32_v*.c
 */

#include <zephyr/init.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/device.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(cad_dma_minimal, CONFIG_CAD_DMA_MINIMAL_LOG_LEVEL);

struct dma_stm32_data {
	uint32_t status;
};

struct dma_stm32_config {
	struct stm32_pclken pclken;
    uint32_t base;
};

static int dma_stm32_init(const struct device *dev)
{
	const struct dma_stm32_config *config = dev->config;
	const struct device *const clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);

	if (!device_is_ready(clk)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	if (clock_control_on(clk,
		(clock_control_subsys_t) &config->pclken) != 0) {
		LOG_ERR("clock op failed\n");
		return -EIO;
	}

	return 0;
}

#define DMA_STM32_INIT_DEV(index)					            \
									                            \
const struct dma_stm32_config dma_stm32_config_##index = {		\
	.pclken = { .bus = DT_INST_CLOCKS_CELL(index, bus),		    \
		    .enr = DT_INST_CLOCKS_CELL(index, bits) },		    \
	.base = DT_INST_REG_ADDR(index),				            \
};									                            \
									                            \
static struct dma_stm32_data dma_stm32_data_##index = {			\
};									                            \
									                            \
DEVICE_DT_INST_DEFINE(index,						            \
		    &dma_stm32_init,					                \
		    NULL,						                        \
		    &dma_stm32_data_##index, &dma_stm32_config_##index,	\
		    PRE_KERNEL_1, CONFIG_DMA_INIT_PRIORITY,		        \
		    NULL)


#if DT_NODE_HAS_STATUS(DT_DRV_INST(0), okay)
DMA_STM32_INIT_DEV(0);
#endif

#if DT_NODE_HAS_STATUS(DT_DRV_INST(1), okay)
DMA_STM32_INIT_DEV(1);
#endif