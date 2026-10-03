add_library(mpu6500 STATIC
    src/mpu6500.cpp
    src/mpu6500_registers.cpp
    src/decode.cpp
    src/mpu6500_calibration.cpp
    src/mpu6500_config.cpp
    src/mpu6500_power.cpp
    src/mpu6500_read.cpp
    src/mpu6500_fifo.cpp
    src/fifo_frame.cpp
)

target_include_directories(mpu6500
    PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(mpu6500 PUBLIC bus pico_float math)

target_compile_options(mpu6500 PRIVATE -Wall -Wextra)
