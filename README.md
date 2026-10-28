# nav_sim

**Stack-agnostic 2D ground-robot simulation platform for dynamic-obstacle navigation.**

Ported from [Intent-MPC](https://github.com/Zhefan-Xu/Intent-MPC) `uav_simulator` (MIT), stripped of all UAV/PX4/livox pieces and re-targeted at differential-drive AGVs in 2D.

## Why separate from your navigation stack

- The simulator is a data factory: it only produces a world + standard ROS2 topics.
- It has zero dependency on any navigation stack (no hanmole_navigation, no Nav2).
- Any stack can consume it by subscribing to the standard topics below.

## What you get

| Feature | Detail |
|---|---|
| Dynamic obstacles | obstacle_path_plugin: waypoint paths, velocity, loop / back-and-forth, sin (least-squares polynomial) motion |
| AGV model | Differential drive, 2D lidar (/scan), RGB-D camera (/camera/depth|color/*), TF + odom |
| Scenes | square_dynamic.world (walls + static box + 3 walking persons), extendable from Intent-MPC worlds |
| Ground truth | ground_truth_publisher: /model_states -> ground-truth detections + poses |
| No lock-in | Only standard ROS2 messages; no custom msg/srv, no nav-stack include |
## Interface contract (what any navigation stack sees)

```
/scan                                sensor_msgs/LaserScan
/odom                                nav_msgs/Odometry
/tf, /tf_static                      base_link -> odom (robot_localization for map)
/camera/depth/image_raw              sensor_msgs/Image
/camera/color/image_raw              sensor_msgs/Image
/camera/depth/camera_info            sensor_msgs/CameraInfo
/model_states                        gazebo_msgs/ModelStates   (ground truth source)
/dynamic_obstacles/ground_truth      vision_msgs/Detection2DArray (opt)
/dynamic_obstacles/ground_truth/poses geometry_msgs/PoseArray  (opt)
/cmd_vel                             geometry_msgs/Twist       (AGV input)
```

## Requirements

- Ubuntu 22.04 / 24.04
- ROS 2 Jazzy
- Gazebo Classic 11 (gazebo_ros_pkgs, gazebo_plugins, gazebo_dev)
- xacro, vision_msgs, cv_bridge

```
sudo apt install ros-jazzy-gazebo-ros-pkgs ros-jazzy-gazebo-ros ros-jazzy-gazebo-plugins ros-jazzy-xacro ros-jazzy-vision-msgs ros-jazzy-cv-bridge
```

## Build

```bash
cd ~/ros2_ws/src
ln -s /path/to/nav_sim .
cd ~/ros2_ws
colcon build --packages-select nav_sim
source install/setup.bash
```

## Run

```bash
ros2 launch nav_sim sim.launch.py publish_ground_truth:=true
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Then start your navigation stack's bringup (it includes sim.launch.py via IncludeLaunchDescription, or just points at these topics).

## Adding a new scene

Copy an Intent-MPC world from worlds/ (they are already 2D floor plans + people), drop the UAV-only models, add libobstacle_path_plugin.so to each moving model:

```xml
<plugin name="obstacle_motion" filename="libobstacle_path_plugin.so">
  <loop>1</loop>
  <velocity>0.6</velocity>
  <path>
    <waypoint>-6 6 0</waypoint>
    <waypoint> 2 6 0</waypoint>
  </path>
</plugin>
```

## License

Ported plugin retains MIT (Intent-MPC / CMU CERLAB). This package: MIT.
