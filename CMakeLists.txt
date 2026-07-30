cmake_minimum_required(VERSION 3.22)
project(ins VERSION 0.0.1 LANGUAGES C CXX ASM)

file(GLOB_RECURSE SOURCES
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc"
)

add_library(ins STATIC ${SOURCES})
target_include_directories(ins
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/internal
)

target_compile_options(ins PRIVATE -O3)

target_link_libraries(ins PUBLIC bsp m)
target_link_libraries(ins PUBLIC components::utils components::math)

set_target_properties(ins PROPERTIES
        C_STANDARD 17
        CXX_STANDARD 23
        CXX_STANDARD_REQUIRED YES
        POSITION_INDEPENDENT_CODE NO
)

add_library(components::ins ALIAS ins)
