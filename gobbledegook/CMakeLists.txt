add_library(ggk "")

find_package(PkgConfig REQUIRED)
pkg_check_modules(GIO REQUIRED IMPORTED_TARGET gio-2.0)

find_package(Threads REQUIRED)

target_include_directories(ggk PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_sources(ggk
    PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/DBusInterface.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/DBusMethod.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/DBusObject.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/GattCharacteristic.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/GattDescriptor.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/GattInterface.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/GattProperty.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/GattService.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Gobbledegook.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/HciAdapter.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/HciSocket.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Init.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Logger.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Mgmt.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Server.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/ServerUtils.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Utils.cpp"
)

target_link_libraries(ggk PkgConfig::GIO Threads::Threads)