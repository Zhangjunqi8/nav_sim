#ifndef NAV_SIM__MECANUM_DRIVE_PLUGIN_HPP_
#define NAV_SIM__MECANUM_DRIVE_PLUGIN_HPP_

// 4-wheel mecanum drive plugin for Gazebo Classic (ROS 2 / gazebo_ros).
//
// Subscribes geometry_msgs/Twist on /cmd_vel, converts to four wheel
// angular velocities with standard mecanum inverse kinematics, and
// publishes nav_msgs/Odometry on /odom plus odom->base_link TF.
//
// Mecanum inverse kinematics (rollers at +/-45deg, REP-103):
//   w_fl = ( vx - vy - (lx + ly) * wz ) / r
//   w_fr = ( vx + vy + (lx + ly) * wz ) / r
//   w_rl = ( vx + vy - (lx + ly) * wz ) / r
//   w_rr = ( vx - vy + (lx + ly) * wz ) / r

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <string>

namespace gazebo
{
  class MecanumDrivePlugin : public ModelPlugin
  {
  public:
    MecanumDrivePlugin() = default;
    void Load(physics::ModelPtr parent_, sdf::ElementPtr sdf_) override;

  private:
    void OnUpdate();
    void OnTwist(const geometry_msgs::msg::Twist::SharedPtr msg);

    physics::ModelPtr model_;
    physics::JointPtr fl_joint_, fr_joint_, rl_joint_, rr_joint_;

    gazebo_ros::Node::SharedPtr ros_node_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    event::ConnectionPtr update_connection_;

    double wheel_radius_{0.1};
    double half_wheelbase_{0.25};
    double half_track_{0.20};
    double max_wheel_torque_{20.0};
    std::string odom_frame_{"odom"};
    std::string robot_base_frame_{"base_link"};

    // OnUpdate runs at the physics rate (1 kHz). Publishing odom/TF on every
    // step floods DDS (measured ~500 Hz odom / ~800 Hz tf) and starves Nav2.
    // Wheel commands still run every step; only the ROS outputs are throttled.
    double publish_rate_{50.0};
    double publish_period_{0.02};

    double vx_{0.0}, vy_{0.0}, wz_{0.0};
    bool command_received_{false};

    ignition::math::Pose3d pose_;
    double yaw_{0.0};
    ignition::math::Vector3d lin_vel_world_;
    common::Time last_update_;
    common::Time last_publish_;
  };

  GZ_REGISTER_MODEL_PLUGIN(MecanumDrivePlugin)
}

#endif  // NAV_SIM__MECANUM_DRIVE_PLUGIN_HPP_
