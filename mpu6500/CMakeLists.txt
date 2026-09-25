add_library(mpu6500 STATIC
    src/mpu6500.cpp
)

target_include_directories(mpu6500
    PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(mpu6500 PUBLIC bus)

target_compile_options(mpu6500 PRIVATE -Wall -Wextra)
