add_library(math INTERFACE)

target_include_directories(math INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(math INTERFACE cxx_std_20)
