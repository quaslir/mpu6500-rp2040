add_executable(wom
    main.cpp
)

target_link_libraries(wom PRIVATE
    mpu6500
    bus_pico
    pico_stdlib
    hardware_spi
)

target_compile_definitions(wom PRIVATE IMU_BUS_${IMU_BUS})

target_compile_options(wom PRIVATE -Wall -Wextra)
target_include_directories(wom PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../common)
pico_enable_stdio_usb(wom 1)
pico_enable_stdio_uart(wom 0)

pico_add_extra_outputs(wom)
