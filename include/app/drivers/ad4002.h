#ifndef ZEPHYR_INCLUDE_DRIVERS_AD4002_
#define ZEPHYR_INCLUDE_DRIVERS_AD4002_

#include <zephyr/device.h>

typedef int (*ad4002_api_continuous_read)(const struct device *dev_master, const struct device *dev_slave, uint16_t* rx_buffer_1, uint16_t* rx_buffer_2, const uint32_t N_samples);
typedef int (*ad4002_api_stop_read)(const struct device *dev);

__subsystem struct ad4002_driver_api {
	ad4002_api_continuous_read continuous_read;
	ad4002_api_stop_read stop_read;
};


static inline int ad4002_continuous_read(const struct device *dev_master, const struct device *dev_slave, uint16_t* rx_buffer_1, uint16_t* rx_buffer_2, const uint32_t N_samples){

	const struct ad4002_driver_api *api = (const struct ad4002_driver_api *)dev_master->api;

	return api->continuous_read(dev_master, dev_slave, rx_buffer_1, rx_buffer_2, N_samples);
}

static inline int ad4002_stop_read(const struct device *dev){

	const struct ad4002_driver_api *api = (const struct ad4002_driver_api *)dev->api;

	return api->stop_read(dev);
}

static int spi_dma_setup(const struct device *dev, uint16_t* rx_buffer, const uint32_t N_samples);

#endif