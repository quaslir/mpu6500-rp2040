add_library(bus INTERFACE)

target_include_directories(bus INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(bus INTERFACE cxx_std_20)
