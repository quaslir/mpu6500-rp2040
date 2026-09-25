add_library(bus_pico STATIC
    src/spi_bus.cpp
    src/i2c_bus.cpp
)

target_include_directories(bus_pico PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(bus_pico PUBLIC
    bus
    pico_stdlib
    hardware_spi
    hardware_i2c
)

target_compile_options(bus_pico PRIVATE -Wall -Wextra)
