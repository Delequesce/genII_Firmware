# Copyright (c) 2021 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

# Out-of-tree drivers for custom classes

# Out-of-tree drivers for existing driver classes

add_subdirectory_ifdef(CONFIG_CAD_STM32_QSPI cad_qspi)