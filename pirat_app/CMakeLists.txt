# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(raket_fw)

include(${ZEPHYR_BASE}/samples/subsys/usb/common/common.cmake)

add_subdirectory(drivers/tdk/icm20948)
include_directories(drivers/tdk/icm20948)
include_directories(src/lib)
FILE(GLOB app_sources src/*.c src/lib/imu.c src/lib/mass_storage_device.c src/lib/ble.c src/lib/led.c)
target_sources(app PRIVATE ${app_sources})

if(CONFIG_BUILD_WITH_TFM)
  target_include_directories(app PRIVATE
    $<TARGET_PROPERTY:tfm,TFM_BINARY_DIR>/api_ns/interface/include
  )
endif()


