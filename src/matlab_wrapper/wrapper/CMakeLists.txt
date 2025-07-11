cmake_minimum_required(VERSION 3.15)
project(tinympc_codegen)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)  # -fPIC

# Add Eigen include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Eigen
    ${CMAKE_CURRENT_SOURCE_DIR}/tinympc
)

# TinyMPC source files
set(TINY_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/tiny_data.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/tinympc/admm.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/tinympc/tiny_api.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/tinympc/rho_benchmark.cpp
)

# Create tinympc library
add_library(tinympcstatic STATIC ${TINY_SOURCES})
target_include_directories(tinympcstatic PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Eigen
    ${CMAKE_CURRENT_SOURCE_DIR}/tinympc
)

# Create example main executable
add_executable(tiny_codegen_example src/tiny_main.cpp src/tiny_data.cpp)
target_include_directories(tiny_codegen_example PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Eigen
    ${CMAKE_CURRENT_SOURCE_DIR}/tinympc
)
target_link_libraries(tiny_codegen_example LINK_PUBLIC tinympcstatic)
