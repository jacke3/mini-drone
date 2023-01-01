cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(pico-drone C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

if (TARGET tinyusb_device)
    add_library(
        bmi160lib
        bmi160.h
        bmi160_defs.h
        bmi160.c
    )

    add_executable(
            PID
            PID.c
    )

pico_enable_stdio_usb(PID 1)
pico_enable_stdio_uart(PID 0)

pico_add_extra_outputs(PID)

target_link_libraries(PID pico_stdlib hardware_pwm hardware_i2c bmi160lib)

elseif(PICO_ON_DEVICE)
    message(WARNING "not building hello_usb because TinyUSB submodule is not initialized in the SDK")
endif()