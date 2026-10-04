add_executable(interrupts
    main.cpp
)

target_include_directories(interrupts PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../common)

target_link_libraries(interrupts PRIVATE
    mpu6500
    bus_pico
    pico_stdlib
    hardware_i2c
    hardware_spi
)

target_compile_options(interrupts PRIVATE -Wall -Wextra)

pico_enable_stdio_usb(interrupts 1)
pico_enable_stdio_uart(interrupts 0)

pico_add_extra_outputs(interrupts)
