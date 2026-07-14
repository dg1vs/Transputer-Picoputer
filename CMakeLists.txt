# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

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
set(sdkVersion 2.3.0)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(Transputer-Picoputer C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(Transputer-Picoputer 
        Transputer-Picoputer.c 
        transputer_progdata.c
        t4mod/arithmetic.c
        t4mod/fparithmetic.c
        t4mod/p.c
        t4mod/server.c
        t4/s_ldexp.c
        t4/s_ldexpf.c
        t4/s_scalbn.c
        t4/s_scalbnf.c
        t4mod/w_remainder.c
        t4mod/w_sqrt.c 
        
        t4mod/spi_ram.c
        helper/oled096.c 
        )

pico_set_program_name(Transputer-Picoputer "Transputer-Picoputer")
pico_set_program_version(Transputer-Picoputer "0.1")

# Generate PIO header
pico_generate_pio_header(Transputer-Picoputer ${CMAKE_CURRENT_LIST_DIR}/picoputer.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(Transputer-Picoputer 1)
pico_enable_stdio_usb(Transputer-Picoputer 0)

# Add the standard library to the build
target_link_libraries(Transputer-Picoputer
        pico_stdlib
        hardware_clocks
        hardware_spi
        hardware_i2c
        hardware_pio
        )


# Add the standard include files to the build
target_include_directories(Transputer-Picoputer PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/t4mod
        ${CMAKE_CURRENT_LIST_DIR}/helper
        )

pico_add_extra_outputs(Transputer-Picoputer)

