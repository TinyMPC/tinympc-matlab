cmake_minimum_required(VERSION 3.0.0)
project(TinyMPC VERSION 0.2.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include/Eigen)

add_subdirectory(src/tinympc)
add_subdirectory(examples)
add_subdirectory(test)

# Set the output folder where your program will be created
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/out)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/out)

find_package(Matlab)
matlab_add_mex(NAME tinympc_mex SRC tinympc_mex.cpp src/tinympc/codegen.hpp src/tinympc/codegen.cpp)