cmake_minimum_required(VERSION 3.15)
project(tinympc_codegen)

include(FetchContent)
FetchContent_Declare(
  tinympc
  GIT_REPOSITORY https://github.com/TinyMPC/TinyMPC.git
  GIT_TAG main
)
FetchContent_MakeAvailable(tinympc)

# Only use generated sources locally
set(GENERATED_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/tiny_data.cpp
    # Add any other generated files here
)

add_executable(tiny_codegen_example ${GENERATED_SRC})
target_include_directories(tiny_codegen_example PUBLIC
    ${tinympc_SOURCE_DIR}/include
    ${tinympc_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_link_libraries(tiny_codegen_example tinympcstatic)
