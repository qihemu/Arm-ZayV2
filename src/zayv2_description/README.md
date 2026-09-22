# zayv2_description

ROS 2 robot description package for the Zay V2 arm. The URDF and STL meshes are
installed under the package share directory.

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select zayv2_description
source install/setup.bash
```

## Launch

Install the joint state publisher GUI for the RViz 2 launch:

```bash
sudo apt-get install ros-humble-joint-state-publisher-gui
```

Display the model in RViz 2 with the joint state publisher GUI:

```bash
ros2 launch zayv2_description display.launch.py
```

Install `ros-humble-gazebo-ros-pkgs` separately to spawn the model in Gazebo
Classic:

```bash
ros2 launch zayv2_description gazebo.launch.py
```

The display launch requires `joint_state_publisher_gui`. The simulation launch
requires `gazebo_ros`. Both are listed as runtime dependencies in `package.xml`.

The joint limits in the URDF should be checked against the hardware before
using the model for motion planning or dynamic simulation.
