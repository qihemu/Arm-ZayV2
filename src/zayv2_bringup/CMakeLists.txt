cmake_minimum_required(VERSION 3.16)
project(zayv2_bringup)

find_package(ament_cmake REQUIRED)

# 单轴和六轴启动参数由同一包安装，避免依赖源码目录路径。
install(DIRECTORY config launch urdf DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
