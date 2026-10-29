#include "nav_sim/mecanum_drive_plugin.hpp"

#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/common/Time.hh>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <memory>

namespace gazebo
{
  void MecanumDrivePlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
  {
    model_ = _parent;

    fl_joint_ = model_->GetJoint(_sdf->Get<std::string>("front_left_joint"));
    fr_joint_ = model_->GetJoint(_sdf->Get<std::string>("front_right_joint"));
    rl_joint_ = model_->GetJoint(_sdf->Get<std::string>("rear_left_joint"));
    rr_joint_ = model_->GetJoint(_sdf->Get<std::string>("rear_right_joint"));
    if (!fl_joint_ || !fr_joint_ || !rl_joint_ || !rr_joint_) {
      gzerr << "MecanumDrivePlugin: one or more joints not found.\n";
      return;
    }

    if (_sdf->HasElement("wheel_radius")) {
      wheel_radius_ = _sdf->Get<double>("wheel_radius");
    }
    if (_sdf->HasElement("wheelbase")) {
      half_wheelbase_ = _sdf->Get<double>("wheelbase") / 2.0;
    }
    if (_sdf->HasElement("track_width")) {
      half_track_ = _sdf->Get<double>("track_width") / 2.0;
    }
    if (_sdf->HasElement("max_wheel_torque")) {
      max_wheel_torque_ = _sdf->Get<double>("max_wheel_torque");
    }
    if (_sdf->HasElement("odom_frame")) {
      odom_frame_ = _sdf->Get<std::string>("odom_frame");
    }
    if (_sdf->HasElement("robot_base_frame")) {
      robot_base_frame_ = _sdf->Get<std::string>("robot_base_frame");
    }

    // ROS 2 node (gazebo_ros) — same mechanism as libgazebo_ros_diff_drive
    ros_node_ = gazebo_ros::Node::Get(_sdf);
    cmd_vel_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(10),
      std::bind(&MecanumDrivePlugin::OnTwist, this, std::placeholders::_1));
    odom_pub_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>(
      "odom", rclcpp::QoS(10));
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(ros_node_);

    pose_ = model_->WorldPose();
    yaw_ = pose_.Rot().Yaw();
    last_update_ = model_->GetWorld()->SimTime();

    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&MecanumDrivePlugin::OnUpdate, this));

    gzmsg << "MecanumDrivePlugin loaded on [" << model_->GetName() << "]\n";
  }

  void MecanumDrivePlugin::OnTwist(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    vx_ = msg->linear.x;
    vy_ = msg->linear.y;
    wz_ = msg->angular.z;
    command_received_ = true;
  }

  void MecanumDrivePlugin::OnUpdate()
  {
    const double sum = half_wheelbase_ + half_track_;
    const double w_fl = ( vx_ - vy_ - sum * wz_) / wheel_radius_;
    const double w_fr = ( vx_ + vy_ + sum * wz_) / wheel_radius_;
    const double w_rl = ( vx_ + vy_ - sum * wz_) / wheel_radius_;
    const double w_rr = ( vx_ - vy_ + sum * wz_) / wheel_radius_;

    // Velocity target + torque *limit* (Gazebo velocity motor idiom).
    // SetForce() would apply a constant torque every step and drive the robot
    // even with no cmd_vel.
    fl_joint_->SetParam("vel", 0, w_fl);
    fr_joint_->SetParam("vel", 0, w_fr);
    rl_joint_->SetParam("vel", 0, w_rl);
    rr_joint_->SetParam("vel", 0, w_rr);
    fl_joint_->SetParam("fmax", 0, max_wheel_torque_);
    fr_joint_->SetParam("fmax", 0, max_wheel_torque_);
    rl_joint_->SetParam("fmax", 0, max_wheel_torque_);
    rr_joint_->SetParam("fmax", 0, max_wheel_torque_);

    // Odometry source = Gazebo model true world pose (ground truth in sim).
    // Exact for navigation validation; wheel command is still applied above.
    const common::Time now = model_->GetWorld()->SimTime();
    pose_ = model_->WorldPose();
    yaw_ = pose_.Rot().Yaw();
    last_update_ = now;

    // Publish odom
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = ros_node_->now();
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = robot_base_frame_;
    odom.pose.pose.position.x = pose_.Pos().X();
    odom.pose.pose.position.y = pose_.Pos().Y();
    odom.pose.pose.position.z = pose_.Pos().Z();
    odom.pose.pose.orientation.x = pose_.Rot().X();
    odom.pose.pose.orientation.y = pose_.Rot().Y();
    odom.pose.pose.orientation.z = pose_.Rot().Z();
    odom.pose.pose.orientation.w = pose_.Rot().W();
    // Twist in the robot frame (REP-145): rotate world velocity by -yaw.
    const ignition::math::Vector3d vel_world = model_->WorldLinearVel();
    const double c = std::cos(yaw_);
    const double s = std::sin(yaw_);
    odom.twist.twist.linear.x = c * vel_world.X() + s * vel_world.Y();
    odom.twist.twist.linear.y = -s * vel_world.X() + c * vel_world.Y();
    odom.twist.twist.angular.z = model_->WorldAngularVel().Z();
    odom_pub_->publish(odom);

    // Publish odom -> base_link TF
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = odom.header.stamp;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = robot_base_frame_;
    tf.transform.translation.x = pose_.Pos().X();
    tf.transform.translation.y = pose_.Pos().Y();
    tf.transform.translation.z = pose_.Pos().Z();
    tf.transform.rotation.x = pose_.Rot().X();
    tf.transform.rotation.y = pose_.Rot().Y();
    tf.transform.rotation.z = pose_.Rot().Z();
    tf.transform.rotation.w = pose_.Rot().W();
    tf_broadcaster_->sendTransform(tf);
  }
}
