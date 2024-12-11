#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ble_sensor)

# NORDIC SDK APP START
target_sources(app PRIVATE
  src/main.c
  src/tk2700/tk2700.c
  src/lis2dh12tr/lis2dh12tr.c
  src/lis2dh12tr/lis2dh12_reg.c
)

# NORDIC SDK APP END
zephyr_library_include_directories(. src/lis2dh12tr src/tk2700)
