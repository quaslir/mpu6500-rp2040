add_executable(filters
    main.cpp
)

target_link_libraries(filters PRIVATE
    mpu6500
    bus_pico
    pico_stdlib
    hardware_spi
)

target_compile_definitions(filters PRIVATE IMU_BUS_${IMU_BUS})

target_compile_options(filters PRIVATE -Wall -Wextra)
target_include_directories(filters PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../common)
pico_enable_stdio_usb(filters 1)
pico_enable_stdio_uart(filters 0)

pico_add_extra_outputs(filters)
