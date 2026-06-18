cmake_minimum_required(VERSION 3.8)
project(arm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 控制节点可执行文件
add_executable(arm_driver_node src/arm_driver_node.cpp)
target_include_directories(arm_driver_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(arm_driver_node
    rclcpp
    moveit_ros_planning_interface
    moveit_msgs
    geometry_msgs
    robot_interfaces
    tf2_geometry_msgs)
target_link_libraries(arm_driver_node
    moveit_ros_planning_interface::moveit_move_group_interface
    moveit_core::moveit_robot_trajectory
    moveit_core::moveit_trajectory_processing
    robot_interfaces::robot_interfaces__rosidl_typesupport_cpp)

install(TARGETS arm_driver_node
    RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
    DESTINATION include)

install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
