cmake_minimum_required(VERSION 3.8)
project(lidar_slam)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Threads REQUIRED)

# 保留厂家SDK的独立边界；只提供一个驱动进程，避免重复占用串口。
set(SDK vendor/rplidar_ros/sdk)
file(GLOB SDK_SOURCES CONFIGURE_DEPENDS
    ${SDK}/src/*.cpp ${SDK}/src/hal/*.cpp
    ${SDK}/src/arch/linux/*.cpp ${SDK}/src/dataunpacker/*.cpp
    ${SDK}/src/dataunpacker/unpacker/*.cpp)
add_executable(c1_driver vendor/rplidar_ros/src/rplidar_node.cpp ${SDK_SOURCES})
target_compile_features(c1_driver PRIVATE cxx_std_14)
target_include_directories(c1_driver PRIVATE ${SDK}/include ${SDK}/src)
target_link_libraries(c1_driver Threads::Threads)
ament_target_dependencies(c1_driver rclcpp sensor_msgs std_srvs)
install(TARGETS c1_driver DESTINATION lib/${PROJECT_NAME})

# Python处理层与文件IO分进程，磁盘写入不能阻塞扫描状态更新。
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/scan_processor scripts/data_manager scripts/map_catalog
    scripts/wheel_odom_tf scripts/nav2_export scripts/config_export DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch rviz docs DESTINATION share/${PROJECT_NAME})
# 与robot_wheel_control一致：配置唯一源文件位于工作空间根config。
install(FILES ../../config/lidar_slam.yaml DESTINATION share/${PROJECT_NAME}/config)
install(FILES README.md vendor/rplidar_ros/LICENSE DESTINATION share/${PROJECT_NAME})

ament_package()
