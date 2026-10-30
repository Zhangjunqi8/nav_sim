#ifndef NAV_SIM__PLANAR_DRIVE_PLUGIN_HPP_
#define NAV_SIM__PLANAR_DRIVE_PLUGIN_HPP_

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

namespace gazebo
{
  // Navigation-oriented drive for a rigid chassis on a flat floor. ODE still
  // solves x/y/yaw contacts; only height, roll and pitch are projected afterward.
  class PlanarDrivePlugin : public ModelPlugin
  {
  public:
    ~PlanarDrivePlugin() override;
    void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;
    void Reset() override;

  private:
    void OnTwist(const geometry_msgs::msg::Twist::SharedPtr msg);
    void OnUpdateBegin();
    void OnUpdateEnd();
    void ClearCommand();
    void PublishOdometry(const common::Time & time);

    physics::ModelPtr model_;
    gazebo_ros::Node::SharedPtr ros_node_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    event::ConnectionPtr begin_connection_;
    event::ConnectionPtr end_connection_;

    std::mutex command_mutex_;
    ignition::math::Vector3d command_{0.0, 0.0, 0.0};
    std::chrono::steady_clock::time_point last_command_;
    bool command_received_{false};
    double cmd_vel_timeout_{0.5};
    double publish_rate_{50.0};
    double publish_period_{0.02};
    double plane_height_{0.15};
    std::string odom_frame_{"odom"};
    std::string robot_base_frame_{"base_footprint"};
    common::Time last_update_;
    common::Time last_publish_;
  };
}

#endif  // NAV_SIM__PLANAR_DRIVE_PLUGIN_HPP_
