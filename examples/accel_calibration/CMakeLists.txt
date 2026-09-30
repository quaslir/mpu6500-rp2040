add_executable(accel_calibration
    main.cpp
)

target_include_directories(accel_calibration PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../common)

target_link_libraries(accel_calibration PRIVATE
    mpu6500
    bus_pico
    pico_stdlib
    hardware_i2c
)

target_compile_options(accel_calibration PRIVATE -Wall -Wextra)

pico_enable_stdio_usb(accel_calibration 1)
pico_enable_stdio_uart(accel_calibration 0)

pico_add_extra_outputs(accel_calibration)
