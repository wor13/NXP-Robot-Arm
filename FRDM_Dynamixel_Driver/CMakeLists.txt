
# Copyright 2026 NXP
#
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.22.0)

cmake_path(APPEND conf_file_path ${CMAKE_CURRENT_LIST_DIR} ${board}_${core_id} prj.conf)
set(CONF_FILE ${conf_file_path} ${CONF_FILE})
include(${SdkRootDirPath}/cmake/extension/mcux.cmake)

project(frdmmcxn947_lpflexcomm_interrupt_transfer_cm33_core0 LANGUAGES C CXX ASM PROJECT_BOARD_PORT_PATH ${board_root}/${board}/driver_examples/lpflexcomm/interrupt_transfer)

include(${SdkRootDirPath}/CMakeLists.txt)


include("${board}_${core_id}/board_files.cmake")

mcux_add_source(
    SOURCES dxl_step1_main.c
	dxl_protocol.c
        dxl_port.c
        dxl.c
        fake_servo.c
)

mcux_add_include(
    INCLUDES cm33_core0
             interrupt_transfer
)
mcux_add_macro(
    CC "-DLPFLEXCOMM_INIT_NOT_USED_IN_DRIVER=1"
)

mcux_convert_binary(BINARY ${APPLICATION_BINARY_DIR}/${MCUX_SDK_PROJECT_NAME}.bin)

mcux_add_include(INCLUDES .)
