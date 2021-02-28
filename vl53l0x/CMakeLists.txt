add_library(vl53l0x "")

target_include_directories(vl53l0x PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_sources(vl53l0x PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/I2Cdev.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/VL53L0X.cpp"
)

target_compile_options(vl53l0x PUBLIC
    -Wformat=2
    -Wall
    -Wextra
    -O2
)