cmake_minimum_required(VERSION 3.15)
project(tinympc_matlab_mex)

include(FetchContent)
FetchContent_Declare(
  tinympc
  GIT_REPOSITORY https://github.com/TinyMPC/TinyMPC.git
  GIT_TAG main
)
FetchContent_MakeAvailable(tinympc)

find_package(Matlab REQUIRED)

set(SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/matlab_wrapper/bindings.cpp
    ${tinympc_SOURCE_DIR}/src/tinympc/tiny_api.cpp
    ${tinympc_SOURCE_DIR}/src/tinympc/admm.cpp
    ${tinympc_SOURCE_DIR}/src/tinympc/codegen.cpp
    ${tinympc_SOURCE_DIR}/src/tinympc/rho_benchmark.cpp
)

matlab_add_mex(NAME tinympc_matlab SRC ${SRC})
target_include_directories(tinympc_matlab PRIVATE
    ${tinympc_SOURCE_DIR}/include
    ${tinympc_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/matlab_wrapper
)
target_link_libraries(tinympc_matlab tinympcstatic)
