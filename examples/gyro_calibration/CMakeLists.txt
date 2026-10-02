add_executable(gyro_calibration
    main.cpp
)

target_include_directories(gyro_calibration PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../common)

target_link_libraries(gyro_calibration PRIVATE
    mpu6500
    bus_pico
    pico_stdlib
    hardware_i2c
    hardware_spi
)

target_compile_options(gyro_calibration PRIVATE -Wall -Wextra)

pico_enable_stdio_usb(gyro_calibration 1)
pico_enable_stdio_uart(gyro_calibration 0)

pico_add_extra_outputs(gyro_calibration)
