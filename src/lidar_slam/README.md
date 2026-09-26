# lidar_slam

C1采集、扫描平面的障碍观测、建图/定位接入与离线资料管理。ROS 2 Humble。

配置统一位于工作空间根`config/lidar_slam.yaml`，与`robot_wheel_control.yaml`同级；包内不保留另一套配置。详见[配置说明](docs/configuration.md)和[Nav2接口契约](docs/nav2_interface.md)。

完整架构、坐标/数据契约、多楼层和实施阶段见 [长期架构方案](docs/architecture.md)；命令见 [运行与数据操作](docs/operations.md)。

默认只启动雷达、感知与文件服务，不启动机械臂、底盘或导航，不输出速度。C1是二维雷达：`PointCloud2`/PCD转换不会增加高度信息；`CLEAR`只是当帧扫描平面的观测，不是整车行驶许可。安装外参和整车外廓未标定时状态为`UNKNOWN`。

```bash
bash build_robot.sh lidar
source scripts/lidar_env.bash
ros2 launch lidar_slam c1.launch.py serial_port:=/dev/rplidar
```

自定义接口统一在`robot_interfaces`：`LidarState`、`GetLidarState`、`SaveLidarCloud`、`LoadLidarReference`。标准扫描、点云、里程计、地图和第三方SLAM接口继续使用上游标准类型。

`vendor/rplidar_ros/`保留厂商SDK和BSD许可，迁移源及原始文件哈希见`docs/migration_manifest.json`；局部驱动修复见架构文档。`docs/C1/`为用户提供的三份原始PDF。运行生成数据默认放在`~/.local/share/arm_zay/lidar_slam/`，或由`LIDAR_SLAM_DATA_ROOT`指定，不写进源码或安装目录。

统一编译、启动和按包查看日志，见[工作空间脚本说明](../../scripts/README.md)；三个入口位于工作空间根目录。
