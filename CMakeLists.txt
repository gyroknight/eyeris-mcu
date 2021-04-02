cmake_minimum_required(VERSION 3.13)

project(eyeris VERSION 0.1
               DESCRIPTION "Main application for Eyeris smart cane"
               LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE Debug)

find_package(Threads REQUIRED)
find_package(ALSA REQUIRED)

add_subdirectory(drv2605l)
add_subdirectory(vl53l0x)

add_executable(eyeris "")
target_include_directories(eyeris PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_sources(eyeris
    PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/AudioController.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/EyerisController.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cc"
)

target_link_libraries(eyeris drv2605l-user-driver vl53l0x Threads::Threads wiringPi ALSA::ALSA)

target_compile_options(eyeris
    PUBLIC -Wall
)
