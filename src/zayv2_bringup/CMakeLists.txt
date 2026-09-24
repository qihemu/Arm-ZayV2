cmake_minimum_required(VERSION 3.16)
project(zayv2_bringup)

find_package(ament_cmake REQUIRED)

# 单轴和六轴启动参数由同一包安装，避免依赖源码目录路径。
install(DIRECTORY config launch urdf DESTINATION share/${PROJECT_NAME})
# 故障监视器独立于 controller_manager 运行，退出时由 launch 停止整套控制进程。
install(PROGRAMS
    scripts/hardware_state_guard.py
    scripts/damiao_slider_test_gui.py
    DESTINATION lib/${PROJECT_NAME}
)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(hardware_state_guard_test test/test_hardware_state_guard.py)
    ament_add_pytest_test(damiao_slider_test_gui_test test/test_damiao_slider_test_gui.py)
endif()

ament_package()
