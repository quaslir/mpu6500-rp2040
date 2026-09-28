add_executable(read_data
    main.cpp
)

target_link_libraries(read_data PRIVATE
    mpu6500
    bus_pico
    pico_stdlib
    hardware_spi
)

target_compile_definitions(read_data PRIVATE IMU_BUS_${IMU_BUS})

target_compile_options(read_data PRIVATE -Wall -Wextra)
target_include_directories(read_data PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../common)
pico_enable_stdio_usb(read_data 1)
pico_enable_stdio_uart(read_data 0)

pico_add_extra_outputs(read_data)
