zephyr_library()

zephyr_library_sources_ifdef(CONFIG_FAST_PWM pwm_stm32_custom_fast.c)