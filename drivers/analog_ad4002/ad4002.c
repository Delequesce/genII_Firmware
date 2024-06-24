#define DT_DRV_COMPAT analog_ad4002

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/dt-bindings/spi/spi.h>
#include <app/drivers/ad4002.h>

#define LOG_LEVEL CONFIG_AD4002_LOG_LEVEL

LOG_MODULE_REGISTER(analog_ad4002);

#define SPI_FRAME_BITS 16

#define SPI_OPER(index) (SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(SPI_FRAME_BITS))

struct ad4002_config{
	struct spi_dt_spec spi_spec;
};


static int analog_ad4002_read(const struct device *dev, const struct spi_buf_set *rx_buffer){

	int ret;
	const struct ad4002_config *cfg = dev->config;
	
	ret = spi_read_dt(&cfg->spi_spec, rx_buffer);
	return ret; 
}

static int ad4002_init(const struct device *dev){
	
	const struct ad4002_config *cfg = dev->config;
	
	if (!spi_is_ready_dt(&cfg->spi_spec)) {
		LOG_ERR("SPI device %s not ready", cfg->spi_spec.bus->name);
		return -ENODEV;
	}
	// Set SPI bus clock frequency to frequency in AD4002 node
	//cfg->bus.config.frequency = cfg->clock_speed;

	return 0;
}

static const struct ad4002_driver_api ad4002_api = {
	.read = analog_ad4002_read,
};

#define AD4002_DEVICE_INIT(index) \
	static const struct ad4002_config ad4002_cfg_##index = { \
		.spi_spec = SPI_DT_SPEC_INST_GET(index, SPI_OPER(index), 0), \
	}; \
\
DEVICE_DT_INST_DEFINE(index, &ad4002_init, NULL, \
	NULL, &ad4002_cfg_##index, POST_KERNEL,      \
	CONFIG_AD4002_SPI_INIT_PRIORITY, &ad4002_api);

// Initialize each active device
DT_INST_FOREACH_STATUS_OKAY(AD4002_DEVICE_INIT)