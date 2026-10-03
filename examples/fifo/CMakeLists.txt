add_executable(fifo
    main.cpp
)

target_link_libraries(fifo PRIVATE
    mpu6500
    bus_pico
    pico_stdlib
    hardware_spi
    hardware_i2c
)

target_compile_definitions(fifo PRIVATE IMU_BUS_${IMU_BUS})

target_compile_options(fifo PRIVATE -Wall -Wextra)
target_include_directories(fifo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../common)
pico_enable_stdio_usb(fifo 1)
pico_enable_stdio_uart(fifo 0)

pico_add_extra_outputs(fifo)
