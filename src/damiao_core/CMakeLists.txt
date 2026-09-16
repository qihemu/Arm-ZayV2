cmake_minimum_required(VERSION 3.16)
project(damiao_core VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
find_package(Threads REQUIRED)

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(FATAL_ERROR "damiao_core SocketCAN backend requires Linux")
endif()

# 核心库可独立构建，不引入 ROS 或 ament 依赖。
add_library(damiao_core SHARED
    src/protocol.cpp
    src/socket_can_transport.cpp
    src/registers.cpp
    src/bus.cpp
)
add_library(damiao_core::damiao_core ALIAS damiao_core)
target_compile_features(damiao_core PUBLIC cxx_std_17)
target_link_libraries(damiao_core PRIVATE Threads::Threads)
target_include_directories(damiao_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
set_target_properties(damiao_core PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 0
    CXX_EXTENSIONS OFF
)

# 导出可重定位的安装接口，供独立程序和后续 ROS 适配层链接。
set(damiao_core_config_dir "${CMAKE_INSTALL_LIBDIR}/cmake/damiao_core")
configure_package_config_file(
    cmake/damiao_coreConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/damiao_coreConfig.cmake
    INSTALL_DESTINATION ${damiao_core_config_dir}
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/damiao_coreConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMinorVersion
)
install(TARGETS damiao_core EXPORT damiao_coreTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT damiao_coreTargets
    NAMESPACE damiao_core::
    DESTINATION ${damiao_core_config_dir}
)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/damiao_coreConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/damiao_coreConfigVersion.cmake
    DESTINATION ${damiao_core_config_dir}
)
install(FILES package.xml README.md DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})

# 软件测试不创建 CAN 接口，也不向实机发送报文。
include(CTest)
if(BUILD_TESTING)
    add_executable(damiao_core_tests test/core_tests.cpp)
    target_link_libraries(damiao_core_tests PRIVATE damiao_core::damiao_core Threads::Threads)
    target_include_directories(damiao_core_tests PRIVATE src)
    add_test(NAME damiao_core_software COMMAND damiao_core_tests)
    set_tests_properties(damiao_core_software PROPERTIES TIMEOUT 30)
    add_executable(damiao_core_vcan_tests test/vcan_tests.cpp)
    target_link_libraries(damiao_core_vcan_tests PRIVATE damiao_core::damiao_core)
    add_test(NAME damiao_core_vcan COMMAND damiao_core_vcan_tests)
    set_tests_properties(damiao_core_vcan PROPERTIES TIMEOUT 10 SKIP_RETURN_CODE 77)
endif()
