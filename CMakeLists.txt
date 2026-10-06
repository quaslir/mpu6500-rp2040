cmake_minimum_required(VERSION 3.13)

if (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(MPU6500_IS_TOP_LEVEL ON)
else()
    set(MPU6500_IS_TOP_LEVEL OFF)
endif()

if (MPU6500_IS_TOP_LEVEL)
    set(PICO_BOARD pico CACHE STRING "Target board")
    include(pico_sdk_import.cmake)
endif()

project(mpu6500_rp2040
    VERSION 1.0.1
    LANGUAGES C CXX ASM
)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(MPU6500_BUILD_EXAMPLES "Build the mpu6500-rp2040 example programs" ${MPU6500_IS_TOP_LEVEL})

if (MPU6500_IS_TOP_LEVEL)
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
    pico_sdk_init()
elseif (NOT TARGET pico_stdlib)
    message(FATAL_ERROR
        "mpu6500-rp2040: the Pico SDK is not initialised. "
        "Call pico_sdk_init() in your project before adding this library.")
endif()

if (PICO_SDK_VERSION_STRING VERSION_LESS "2.0.0")
    message(FATAL_ERROR "Pico SDK 2.0.0 or newer is required (found ${PICO_SDK_VERSION_STRING})")
endif()

if (MPU6500_IS_TOP_LEVEL)
    find_program(CLANG_FORMAT clang-format)

    if (CLANG_FORMAT)
        file(GLOB_RECURSE FORMAT_SOURCES CONFIGURE_DEPENDS
            ${PROJECT_SOURCE_DIR}/lib/*.hpp      ${PROJECT_SOURCE_DIR}/lib/*.cpp
            ${PROJECT_SOURCE_DIR}/mpu6500/*.hpp  ${PROJECT_SOURCE_DIR}/mpu6500/*.cpp
            ${PROJECT_SOURCE_DIR}/examples/*.hpp ${PROJECT_SOURCE_DIR}/examples/*.cpp
        )

        add_custom_target(format
            COMMAND ${CLANG_FORMAT} -i ${FORMAT_SOURCES}
            COMMENT "Formatting sources with clang-format"
            VERBATIM
        )

        add_custom_target(format-check
            COMMAND ${CLANG_FORMAT} --dry-run --Werror ${FORMAT_SOURCES}
            COMMENT "Checking formatting with clang-format"
            VERBATIM
        )
    else()
        message(STATUS "clang-format not found: 'format' targets disabled")
    endif()
endif()

add_subdirectory(lib/math)
add_subdirectory(lib/bus)
add_subdirectory(lib/bus_pico)
add_subdirectory(lib/orientation)
add_subdirectory(mpu6500)

if (MPU6500_BUILD_EXAMPLES)
    set(IMU_BUS "SPI" CACHE STRING "Bus for the MPU6500 in the examples: SPI or I2C")
    set_property(CACHE IMU_BUS PROPERTY STRINGS SPI I2C)
    if (NOT IMU_BUS MATCHES "^(SPI|I2C)$")
        message(FATAL_ERROR "IMU_BUS must be SPI or I2C (got '${IMU_BUS}')")
    endif()
    message(STATUS "MPU6500 bus for examples: ${IMU_BUS}")

    add_subdirectory(examples)
endif()
