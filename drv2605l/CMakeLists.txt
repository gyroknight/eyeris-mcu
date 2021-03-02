add_library(drv2605l-user-driver "")
target_include_directories(drv2605l-user-driver PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_sources(drv2605l-user-driver
    PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/drv2605lDriver.cc"
)

target_link_libraries(drv2605l-user-driver i2c)

target_compile_options(drv2605l-user-driver
    PUBLIC -Wall -Wno-unused-const-variable
)
