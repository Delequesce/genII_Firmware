zephyr_library()

zephyr_library_sources_ifdef(CONFIG_AD4002 ad4002.c)
