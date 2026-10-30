cmake_minimum_required(VERSION 3.16)
project(nav_sim)

# ---- ROS2 / Gazebo (Classic) ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# ---- Dynamic obstacle path plugin (ported from Intent-MPC uav_simulator, MIT) ----
add_library(obstacle_path_plugin SHARED
  src/obstacle_path_plugin.cpp
)
target_include_directories(obstacle_path_plugin PUBLIC
  include
)
target_link_libraries(obstacle_path_plugin
  ${GAZEBO_LIBRARIES}
  Eigen3::Eigen
)
ament_target_dependencies(obstacle_path_plugin
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  nav_msgs
  gazebo_ros
)
target_compile_features(obstacle_path_plugin PUBLIC cxx_std_17)

# ---- Mecanum 4-wheel drive plugin (cmd_vel -> 4 wheel speeds + odom) ----
add_library(mecanum_drive_plugin SHARED
  src/mecanum_drive_plugin.cpp
)
target_include_directories(mecanum_drive_plugin PUBLIC
  include
)
target_link_libraries(mecanum_drive_plugin
  ${GAZEBO_LIBRARIES}
  Eigen3::Eigen
)
ament_target_dependencies(mecanum_drive_plugin
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  gazebo_ros
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_compile_features(mecanum_drive_plugin PUBLIC cxx_std_17)

# ---- Planar drive for the rigid navigation AGV ----
add_library(planar_drive_plugin SHARED
  src/planar_drive_plugin.cpp
)
target_include_directories(planar_drive_plugin PUBLIC
  include
)
target_link_libraries(planar_drive_plugin
  ${GAZEBO_LIBRARIES}
)
ament_target_dependencies(planar_drive_plugin
  rclcpp
  geometry_msgs
  nav_msgs
  gazebo_ros
  tf2_ros
)
target_compile_features(planar_drive_plugin PUBLIC cxx_std_17)

# ---- Ground-truth publisher (/model_states -> /dynamic_obstacles/ground_truth) ----
add_executable(ground_truth_publisher
  src/ground_truth_publisher.cpp
)
ament_target_dependencies(ground_truth_publisher
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  gazebo_msgs
  vision_msgs
)
target_compile_features(ground_truth_publisher PUBLIC cxx_std_17)

# ---- Install ----
install(TARGETS obstacle_path_plugin mecanum_drive_plugin planar_drive_plugin ground_truth_publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch worlds models urdf rviz
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS test/planar_drive_test.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
