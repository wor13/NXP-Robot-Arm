# This is a separate application: its main() replaces neither camera example.
cmake_minimum_required(VERSION 3.22.0)
include(${SdkRootDirPath}/cmake/extension/mcux.cmake)

# Reapply our choices after hello_world's board configuration is merged.
set(CONF_FILE "${CMAKE_CURRENT_LIST_DIR}/prj.conf")

# Reuse the SDK's FRDM board integration, startup, linker script and clocks.
# prj.conf selects our own pin and hardware setup instead of hello_world's.
project(vl53l0x_distance LANGUAGES C CXX ASM
    PROJECT_BOARD_PORT_PATH ${board_root}/${board}/demo_apps/hello_world)
include(${SdkRootDirPath}/CMakeLists.txt)

mcux_add_source(BASE_PATH ${CMAKE_CURRENT_LIST_DIR} SOURCES
    main.c hardware_init.c vl53l0x_platform.c
    vendor/st/api/core/src/vl53l0x_api.c
    vendor/st/api/core/src/vl53l0x_api_calibration.c
    vendor/st/api/core/src/vl53l0x_api_core.c
    vendor/st/api/core/src/vl53l0x_api_ranging.c
    vendor/st/api/core/src/vl53l0x_api_strings.c)
mcux_add_include(BASE_PATH ${CMAKE_CURRENT_LIST_DIR}
    INCLUDES . vendor/st/api/core/inc)

# Apply to the driver too. Its default of zero waits forever on stuck flags.
# This bounds polling iterations; it is NOT a timeout measured in milliseconds.
mcux_add_macro(CC "I2C_RETRY_TIMES=100000U")
# Tell ST's DataInit to use its 2.8-V I/O mode (the usual breakout sensor side),
# rather than its default 1.8-V mode. This is not a voltage regulator setting.
mcux_add_macro(CC "USE_I2C_2V8")
mcux_convert_binary(BINARY ${APPLICATION_BINARY_DIR}/${MCUX_SDK_PROJECT_NAME}.bin)
