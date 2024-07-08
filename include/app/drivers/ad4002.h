#ifndef ZEPHYR_INCLUDE_DRIVERS_AD4002_
#define ZEPHYR_INCLUDE_DRIVERS_AD4002_

#include <zephyr/device.h>

typedef int (*ad4002_api_read)(const struct device *dev);
typedef int (*ad4002_api_continuous_read)(const struct device *dev, int16_t* rx_buffer, const uint32_t N_samples);
typedef int (*ad4002_api_stop_read)(const struct device *dev);

__subsystem struct ad4002_driver_api {
	ad4002_api_read read;
	ad4002_api_continuous_read continuous_read;
	ad4002_api_stop_read stop_read;
};

static inline int ad4002_read(const struct device *dev){

	const struct ad4002_driver_api *api = (const struct ad4002_driver_api *)dev->api;

	return api->read(dev);
}

static inline int ad4002_continuous_read(const struct device *dev, int16_t* rx_buffer, const uint32_t N_samples){

	const struct ad4002_driver_api *api = (const struct ad4002_driver_api *)dev->api;

	return api->continuous_read(dev, rx_buffer, N_samples);
}

static inline int ad4002_stop_read(const struct device *dev){

	const struct ad4002_driver_api *api = (const struct ad4002_driver_api *)dev->api;

	return api->stop_read(dev);
}


#endif