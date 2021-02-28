cmake_minimum_required(VERSION 3.13)

project(eyeris VERSION 0.1
                             DESCRIPTION "Main application for Eyeris smart cane"
                             LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE Debug)

add_subdirectory(vl53l0x)

add_executable(eyeris "")
target_include_directories(eyeris PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_sources(eyeris
    PRIVATE
    # "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/continuousMultipleSensors.cpp"
)

target_link_libraries(eyeris i2c wiringPi vl53l0x)

target_compile_options(eyeris
    PUBLIC -Wall -Wno-unused-const-variable
)
