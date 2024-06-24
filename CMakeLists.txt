# Copyright (c) 2021 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0
#
# This CMake file is picked by the Zephyr build system because it is defined
# as the module CMake entry point (see zephyr/module.yml).

cmake_minimum_required(VERSION 3.28)

set(BOARD nucleo_wb55rg)
set(BOARD_FLASH_RUNNER jlink)

set(ZEPHYR_EXTRA_MODULES drivers/cad_qspi)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(app)

target_sources(app PRIVATE app/src/main.c)
add_subdirectory(drivers)

# This is needed so that custom driver classes using system calls are taken into
# account
zephyr_syscall_include_directories(include)

zephyr_include_directories(include)

add_subdirectory(lib)
