cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(deskdisplay)

pico_sdk_init()

add_executable(deskdisplay deskdisplay.cpp dependencies/WS2812.cpp writablearray.cpp debug.cpp)

# enable usb output, disable uart output
pico_enable_stdio_usb(deskdisplay 1)
pico_enable_stdio_uart(deskdisplay 0)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(deskdisplay)

# add url via pico_set_program_url
# example_auto_set_url(deskdisplay)

target_link_libraries(deskdisplay pico_stdlib hardware_pio hardware_flash hardware_sync)

pico_generate_pio_header(deskdisplay ${CMAKE_CURRENT_LIST_DIR}/dependencies/WS2812.pio)