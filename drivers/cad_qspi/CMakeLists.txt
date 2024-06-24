zephyr_library()

zephyr_library_sources_ifdef(CONFIG_CAD_STM32_QSPI cad_stm32_qspi.c)
