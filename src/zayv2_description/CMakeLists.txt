cmake_minimum_required(VERSION 3.8)

project(zayv2_description)

find_package(ament_cmake REQUIRED)

# Install the robot model and ROS 2 launch resources.
install(DIRECTORY launch meshes rviz urdf
    DESTINATION share/${PROJECT_NAME}
    PATTERN "__pycache__" EXCLUDE
)
install(FILES README.md
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
