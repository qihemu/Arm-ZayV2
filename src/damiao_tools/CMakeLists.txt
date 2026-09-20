cmake_minimum_required(VERSION 3.16)
project(damiao_tools VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)
find_package(Threads REQUIRED)
find_package(damiao_core CONFIG REQUIRED)
find_package(yaml-cpp REQUIRED)

# 包内支持库承载配置解析和会话状态机，不作为安装后的公共 C++ API。
add_library(damiao_motor_tool_support STATIC
    src/config.cpp
    src/action_sequence.cpp
    src/motor_scanner.cpp
    src/motor_bus_session.cpp
    src/motor_manager.cpp
    src/console_output.cpp
)
target_compile_features(damiao_motor_tool_support PUBLIC cxx_std_17)
target_link_libraries(damiao_motor_tool_support
    PUBLIC
        damiao_core::damiao_core
        Threads::Threads
        yaml-cpp
)
target_include_directories(damiao_motor_tool_support PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
set_target_properties(damiao_motor_tool_support PROPERTIES
    CXX_EXTENSIONS OFF
)

add_executable(damiao_motor_tool src/main.cpp)
target_link_libraries(damiao_motor_tool PRIVATE damiao_motor_tool_support)

install(TARGETS damiao_motor_tool
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(FILES config/motor.example.yaml config/demo_sequence.txt
    DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/config
)
install(FILES README.md package.xml
    DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}
)

# 软件测试使用内存模拟传输，不创建 SocketCAN 连接。
include(CTest)
if(BUILD_TESTING)
    add_executable(damiao_tools_tests test/tools_tests.cpp)
    target_link_libraries(damiao_tools_tests PRIVATE damiao_motor_tool_support)
    add_test(NAME damiao_tools_software COMMAND damiao_tools_tests)
    set_tests_properties(damiao_tools_software PROPERTIES TIMEOUT 30)
endif()
