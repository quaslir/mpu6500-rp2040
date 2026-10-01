add_library(orientation STATIC
    src/orientation.cpp
)

target_include_directories(orientation PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(orientation PUBLIC
    math
)

target_compile_options(orientation PRIVATE -Wall -Wextra)
