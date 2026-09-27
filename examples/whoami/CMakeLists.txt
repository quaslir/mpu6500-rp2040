add_executable(whoami
    main.cpp
)

target_link_libraries(whoami PRIVATE
    mpu6500
    bus_pico
    pico_stdlib
    hardware_spi
)

target_compile_definitions(whoami PRIVATE IMU_BUS_${IMU_BUS})

target_compile_options(whoami PRIVATE -Wall -Wextra)
target_include_directories(whoami PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../common)
pico_enable_stdio_usb(whoami 1)
pico_enable_stdio_uart(whoami 0)

pico_add_extra_outputs(whoami)
