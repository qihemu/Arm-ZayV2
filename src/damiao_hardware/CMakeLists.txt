cmake_minimum_required(VERSION 3.16)
project(damiao_hardware VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(damiao_core CONFIG REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Threads REQUIRED)

# 插件只实现 ROS 适配，CAN 协议和独占总线复用核心库。
add_library(damiao_hardware SHARED src/damiao_system_hardware.cpp)
target_compile_features(damiao_hardware PUBLIC cxx_std_17)
target_include_directories(damiao_hardware PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(damiao_hardware damiao_core::damiao_core Threads::Threads)
ament_target_dependencies(damiao_hardware
    hardware_interface pluginlib rclcpp rclcpp_lifecycle)
pluginlib_export_plugin_description_file(hardware_interface damiao_hardware_plugins.xml)

install(TARGETS damiao_hardware LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_libraries(damiao_hardware)
ament_export_dependencies(damiao_core hardware_interface pluginlib rclcpp rclcpp_lifecycle)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(damiao_hardware_test test/damiao_system_hardware_test.cpp)
    target_link_libraries(damiao_hardware_test damiao_hardware)
    ament_target_dependencies(damiao_hardware_test hardware_interface pluginlib)
endif()

ament_package()
