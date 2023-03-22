cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(PicoUsbLedStrip)

pico_sdk_init()

add_executable(PicoUsbLedStrip 
    PicoUsbLedStrip.cpp
    dependencies/WS2812.cpp
    writablearray.cpp
    fftData.cpp
)
add_library(kiss_fftr dependencies/kiss_fftr.c)
add_library(kiss_fft dependencies/kiss_fft.c)

target_link_libraries(kiss_fftr kiss_fft)

# enable usb output, disable uart output
pico_enable_stdio_usb(PicoUsbLedStrip 1)
pico_enable_stdio_uart(PicoUsbLedStrip 0)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(PicoUsbLedStrip)

# add url via pico_set_program_url
# example_auto_set_url(PicoUsbLedStrip)

target_link_libraries(PicoUsbLedStrip
    pico_stdlib
    pico_multicore
    hardware_pio
    hardware_adc
    hardware_dma
    hardware_flash
    hardware_sync
    kiss_fftr
)

pico_generate_pio_header(PicoUsbLedStrip ${CMAKE_CURRENT_LIST_DIR}/dependencies/WS2812.pio)