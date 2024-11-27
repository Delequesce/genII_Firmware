#ifndef ZEPHYR_INCLUDE_DRIVERS_AD4002_
#define ZEPHYR_INCLUDE_DRIVERS_AD4002_

#include <zephyr/device.h>

typedef void (*ad4002_irq_callback_user_data_t)(const struct device *dev);

typedef int (*ad4002_api_init_read)(const struct device *dev_master, const struct device *dev_slave, uint16_t* rx_buffer_1, uint16_t* rx_buffer_2, const uint32_t N_samples);
typedef int (*ad4002_api_start_read)(const struct device *dev, const uint32_t N_samples);
typedef int (*ad4002_api_irq_callback_set)(const struct device *dev, ad4002_irq_callback_user_data_t cb);
typedef int (*ad4002_api_shutdown)(const struct device *dev);

__subsystem struct ad4002_driver_api {
	ad4002_api_init_read init_read;
	ad4002_api_start_read start_read;
	ad4002_api_irq_callback_set irq_callback_set;
	ad4002_api_shutdown shutdown;
};


static inline int ad4002_init_read(const struct device *dev_master, const struct device *dev_slave, uint16_t* rx_buffer_1, uint16_t* rx_buffer_2, const uint32_t N_samples){

	const struct ad4002_driver_api *api = (const struct ad4002_driver_api *)dev_master->api;

	return api->init_read(dev_master, dev_slave, rx_buffer_1, rx_buffer_2, N_samples);
}

static inline int ad4002_start_read(const struct device *dev, const uint32_t N_samples){

	const struct ad4002_driver_api *api = (const struct ad4002_driver_api *)dev->api;

	return api->start_read(dev, N_samples);
}

static inline int ad4002_shutdown(const struct device *dev){

	const struct ad4002_driver_api *api = (const struct ad4002_driver_api *)dev->api;

	return api->shutdown(dev);
}


static inline int ad4002_irq_callback_set(const struct device *dev, uint32_t cb){

	const struct ad4002_driver_api *api = (const struct ad4002_driver_api *)dev->api;

	return api->irq_callback_set(dev, cb);
}


static int spi_dma_setup(const struct device *dev, uint16_t* rx_buffer, const uint32_t N_samples, const bool tx_rx);

#endif