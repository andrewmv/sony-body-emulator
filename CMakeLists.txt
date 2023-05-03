cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

project(sony_body_emulator C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()
add_executable(sony_body_emulator
	body_emu.c
)

# Generate PIO headers in build dir
pico_generate_pio_header(sony_body_emulator ${CMAKE_CURRENT_LIST_DIR}/rx-miso.pio)
pico_generate_pio_header(sony_body_emulator ${CMAKE_CURRENT_LIST_DIR}/tx-mosi.pio)

# Enable serial console over UART
pico_enable_stdio_usb(sony_body_emulator 1)
pico_enable_stdio_uart(sony_body_emulator 1)

pico_add_extra_outputs(sony_body_emulator)

target_sources(sony_body_emulator PRIVATE body_emu.c)

target_link_libraries(sony_body_emulator PRIVATE
        pico_stdlib
        hardware_pio
        hardware_dma
        )
