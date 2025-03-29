# SPDX-License-Identifier: Apache-2.0
zephyr_library_add_dependencies(offsets_h)
zephyr_library_sources(icm20948.c)
zephyr_library_include_directories(${ZEPHYR_BASE}/drivers)