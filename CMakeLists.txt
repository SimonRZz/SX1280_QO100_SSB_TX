# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.20)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(SX1280SDR C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# --------------------------------------------------------------------
# TinyUSB (vendored as subdirectory)
# --------------------------------------------------------------------
set(TINYUSB_DIR ${CMAKE_CURRENT_LIST_DIR}/external/tinyusb)
set(TINYUSB_FAMILY_DIR ${TINYUSB_DIR}/hw/bsp/rp2040)

# Add executable
add_executable(SX1280SDR
    main.c
    usb_descriptors.c
    gpsdo.c
)

pico_set_program_name(SX1280SDR "SX1280SDR")
pico_set_program_version(SX1280SDR "0.1")

# UART/USB stdio disabled — GP0/GP1 are used for GPSDO I2C (SI5351).
# All control is via USB CDC (TinyUSB).
pico_enable_stdio_uart(SX1280SDR 0)
pico_enable_stdio_usb(SX1280SDR 0)

# Pico SDK libs
target_link_libraries(SX1280SDR
    pico_stdlib
    pico_unique_id
    hardware_spi
    hardware_gpio
    hardware_clocks
    hardware_i2c
    hardware_uart
    pico_multicore
    hardware_pio
    hardware_pwm
    m
)


# Include paths: projekt + TinyUSB + BSP
target_include_directories(SX1280SDR PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${TINYUSB_DIR}/src
    ${TINYUSB_DIR}/hw
    ${TINYUSB_FAMILY_DIR}
    ${PICO_SDK_PATH}/src/rp2_common/pico_unique_id/include
    # TinyUSB rp2040 board-specific headers (selected board: raspberry_pi_pico2)
    ${TINYUSB_FAMILY_DIR}/boards/raspberry_pi_pico2
)




# TinyUSB compile-time config (kluczowe makra)
target_compile_definitions(SX1280SDR PRIVATE
    CFG_TUSB_MCU=OPT_MCU_RP2040
    CFG_TUSB_OS=OPT_OS_PICO
)

# Link pico hardware libraries required by TinyUSB rp2040 BSP (family.c)
target_link_libraries(SX1280SDR
    pico_sync
    hardware_clocks
    hardware_resets
    hardware_gpio
    hardware_irq
)

# TinyUSB sources (device + audio + rp2040 portable + family BSP)
target_sources(SX1280SDR PRIVATE
    ${TINYUSB_DIR}/src/tusb.c
    ${TINYUSB_DIR}/src/common/tusb_fifo.c
    ${TINYUSB_DIR}/src/device/usbd.c
    ${TINYUSB_DIR}/src/device/usbd_control.c
    ${TINYUSB_DIR}/src/class/audio/audio_device.c
    ${TINYUSB_DIR}/src/class/cdc/cdc_device.c
    ${TINYUSB_DIR}/src/class/hid/hid_device.c
    ${TINYUSB_DIR}/src/portable/raspberrypi/rp2040/dcd_rp2040.c
    ${TINYUSB_DIR}/src/portable/raspberrypi/rp2040/rp2040_usb.c
    ${TINYUSB_FAMILY_DIR}/family.c
)

pico_add_extra_outputs(SX1280SDR)
