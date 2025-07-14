cmake_minimum_required(VERSION 3.15...3.26)

project(tinympc_matlab_mex)

find_package(Matlab REQUIRED)


# --- TinyMPC -------------------------------------------------
add_subdirectory(tinympc/TinyMPC tinympc_build)

set(SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/bindings.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/tinympc/TinyMPC/src/tinympc/tiny_api.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/tinympc/TinyMPC/src/tinympc/admm.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/tinympc/TinyMPC/src/tinympc/codegen.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/tinympc/TinyMPC/src/tinympc/rho_benchmark.cpp
)

matlab_add_mex(NAME tinympc_matlab SRC ${SRC})
target_include_directories(tinympc_matlab PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/tinympc/TinyMPC/include
    ${CMAKE_CURRENT_SOURCE_DIR}/tinympc/TinyMPC/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(tinympc_matlab tinympcstatic)
