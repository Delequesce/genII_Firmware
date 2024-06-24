#ifndef ZEPHYR_INCLUDE_DRIVERS_CAD_QSPI_H_
#define ZEPHYR_INCLUDE_DRIVERS_CAD_QSPI_H_


#include <zephyr/kernel.h>
#include <zephyr/device.h>

//typedef void (*irq_config_func_t)(const struct device *dev);
//static void flash_stm32_qspi_irq_config_func(const struct device *dev);

//#define CAD_QSPI_STM32_EXPORT_API static

#define DMA_GET_CHANNEL_OFFSET(channel)		(8 + (channel-1) * 20)


typedef int (*cad_qspi_api_read)(const struct device *dev, void *data, size_t len);

static void qspi_dma_callback(const struct device *, void *, uint32_t, int);

__subsystem struct cad_qspi_driver_api {
	cad_qspi_api_read read;
};


static inline int qspi_read(const struct device *dev, uint16_t *data, uint16_t size)
{
	struct cad_qspi_driver_api *api = (struct cad_qspi_driver_api *)dev->api;

	//printf("%x", (unsigned int)dev);
	return api->read(dev, data, size);
	return 1;
}




#endif /* ZEPHYR_INCLUDE_DRIVERS_CAD_QSPI_H_ */

