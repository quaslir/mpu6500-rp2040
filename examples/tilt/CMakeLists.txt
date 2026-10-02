add_executable(tilt
    main.cpp
)

target_link_libraries(tilt PRIVATE
    mpu6500
    bus_pico
    pico_stdlib
    hardware_spi
    hardware_i2c
    orientation
)

target_compile_definitions(tilt PRIVATE IMU_BUS_${IMU_BUS})

target_compile_options(tilt PRIVATE -Wall -Wextra)
target_include_directories(tilt PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../common)
pico_enable_stdio_usb(tilt 1)
pico_enable_stdio_uart(tilt 0)

pico_add_extra_outputs(tilt)
