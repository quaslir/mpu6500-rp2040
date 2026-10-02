add_executable(console
    main.cpp
)

target_include_directories(console PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../common)

target_link_libraries(console PRIVATE
    mpu6500
    bus_pico
    pico_stdlib
    hardware_i2c
    hardware_spi
)

target_compile_options(console PRIVATE -Wall -Wextra)

pico_enable_stdio_usb(console 1)
pico_enable_stdio_uart(console 0)

pico_add_extra_outputs(console)
