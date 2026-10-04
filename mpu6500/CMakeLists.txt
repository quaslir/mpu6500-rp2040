add_library(mpu6500 STATIC
    src/driver/lifecycle.cpp
src/driver/measurement.cpp
src/driver/read.cpp
src/driver/power.cpp
src/driver/fifo.cpp
src/driver/interrupts.cpp
src/registers/mpu6500_registers.cpp
src/codec/decode.cpp
src/codec/fifo_frame.cpp
src/calibration.cpp

)

target_include_directories(mpu6500
    PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(mpu6500 PUBLIC bus pico_float math)

target_compile_options(mpu6500 PRIVATE -Wall -Wextra)
