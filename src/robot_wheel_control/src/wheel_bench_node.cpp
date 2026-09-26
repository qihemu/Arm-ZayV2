#include <robot_wheel_control/ros_api.hpp>
#include <iostream>
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try
    {
        auto settings = std::make_shared<rclcpp::Node>("wheel_settings");
        const auto path = settings->declare_parameter<std::string>("config_file", "");
        const auto backend = settings->declare_parameter<std::string>("backend", "");
        // standalone relative不加载diff_drive_controller，避免两个控制源同时写轮速。
        const auto mode = settings->declare_parameter<std::string>("operation_mode", "");
        auto config = robot_wheel_control::load_configuration(path, backend, mode);
        if (config.mode == "base")
        {
            throw std::runtime_error("base requires wheel_base.launch.py controller owner");
        }
        std::cout << "Wheel source=" << config.source_path << " digest=" << config.digest
                  << " mode=" << config.mode << " effective_wheel_limit=" << config.wheel_speed
                  << " action_limit_m=" << config.action_distance << std::endl;
        auto runtime = std::make_shared<robot_wheel_control::WheelRuntime>(config);
        auto api = std::make_shared<robot_wheel_control::WheelRosApi>(runtime);
        runtime->start();
        rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), config.executor_threads);
        executor.add_node(api);
        executor.spin();
        runtime->shutdown();
    }
    catch (const std::exception &e)
    {
        std::cerr << "wheel_bench_node: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
