#include "nav_sim/planar_drive_plugin.hpp"

#include <cmath>
#include <functional>

namespace gazebo
{
  PlanarDrivePlugin::~PlanarDrivePlugin()
  {
    begin_connection_.reset();
    end_connection_.reset();
  }

  void PlanarDrivePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
  {
    model_ = model;
    if (sdf->HasElement("cmd_vel_timeout")) {
      cmd_vel_timeout_ = sdf->Get<double>("cmd_vel_timeout");
    }
    if (sdf->HasElement("publish_rate")) {
      publish_rate_ = sdf->Get<double>("publish_rate");
    }
    if (!std::isfinite(cmd_vel_timeout_) || cmd_vel_timeout_ <= 0.0 ||
      !std::isfinite(publish_rate_) || publish_rate_ <= 0.0)
    {
      gzerr << "PlanarDrivePlugin: timeout and publish rate must be finite and positive.\n";
      return;
    }
    if (sdf->HasElement("odom_frame")) {
      odom_frame_ = sdf->Get<std::string>("odom_frame");
    }
    if (sdf->HasElement("robot_base_frame")) {
      robot_base_frame_ = sdf->Get<std::string>("robot_base_frame");
    }
    // The pose projection assumes the model is one rigid body. Reject a model
    // with moving joints rather than silently fighting its wheel constraints.
    if (model_->GetLinks().size() != 1) {
      gzerr << "PlanarDrivePlugin: all chassis joints must be fixed.\n";
      return;
    }
    publish_period_ = 1.0 / publish_rate_;
    plane_height_ = model_->WorldPose().Pos().Z();
    last_update_ = model_->GetWorld()->SimTime();
    last_publish_ = last_update_;

    ros_node_ = gazebo_ros::Node::Get(sdf);
    cmd_vel_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(1),
      std::bind(&PlanarDrivePlugin::OnTwist, this, std::placeholders::_1));
    odom_pub_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>(
      "odom", rclcpp::QoS(10));
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(ros_node_);
    begin_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&PlanarDrivePlugin::OnUpdateBegin, this));
    end_connection_ = event::Events::ConnectWorldUpdateEnd(
      std::bind(&PlanarDrivePlugin::OnUpdateEnd, this));
    gzmsg << "PlanarDrivePlugin loaded on [" << model_->GetName()
          << "] height=" << plane_height_ << " timeout=" << cmd_vel_timeout_ << "s\n";
  }

  void PlanarDrivePlugin::ClearCommand()
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_.Set(0.0, 0.0, 0.0);
    command_received_ = false;
  }

  void PlanarDrivePlugin::Reset()
  {
    ClearCommand();
    if (model_) {
      model_->SetLinearVel(ignition::math::Vector3d::Zero);
      model_->SetAngularVel(ignition::math::Vector3d::Zero);
      last_update_ = model_->GetWorld()->SimTime();
      last_publish_ = last_update_;
    }
  }

  void PlanarDrivePlugin::OnTwist(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->linear.y) ||
      !std::isfinite(msg->angular.z))
    {
      command_.Set(0.0, 0.0, 0.0);
      command_received_ = false;
      return;
    }
    command_.Set(msg->linear.x, msg->linear.y, msg->angular.z);
    last_command_ = std::chrono::steady_clock::now();
    command_received_ = true;
  }

  void PlanarDrivePlugin::OnUpdateBegin()
  {
    const common::Time now = model_->GetWorld()->SimTime();
    if (now < last_update_) {
      ClearCommand();
      last_publish_ = now;
    }
    last_update_ = now;

    ignition::math::Vector3d command;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      const double age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_command_).count();
      if (!command_received_ || age > cmd_vel_timeout_) {
        command_.Set(0.0, 0.0, 0.0);
        command_received_ = false;
      }
      command = command_;
    }
    const double yaw = model_->WorldPose().Rot().Yaw();
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    model_->SetLinearVel(ignition::math::Vector3d(
      c * command.X() - s * command.Y(), s * command.X() + c * command.Y(), 0.0));
    model_->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, command.Z()));
  }

  void PlanarDrivePlugin::OnUpdateEnd()
  {
    auto pose = model_->WorldPose();
    auto linear = model_->WorldLinearVel();
    auto angular = model_->WorldAngularVel();
    // Keep the contact solver's x/y/yaw. Never integrate a commanded position:
    // doing so would move the robot through walls and corrupt blocked odometry.
    pose.Pos().Z(plane_height_);
    pose.Rot() = ignition::math::Quaterniond(0.0, 0.0, pose.Rot().Yaw());
    model_->SetWorldPose(pose);
    linear.Z(0.0);
    angular.X(0.0);
    angular.Y(0.0);
    model_->SetLinearVel(linear);
    model_->SetAngularVel(angular);

    const common::Time now = model_->GetWorld()->SimTime();
    if ((now - last_publish_).Double() >= publish_period_) {
      last_publish_ = now;
      PublishOdometry(now);
    }
  }

  void PlanarDrivePlugin::PublishOdometry(const common::Time & time)
  {
    const auto pose = model_->WorldPose();
    const auto linear = model_->WorldLinearVel();
    const auto angular = model_->WorldAngularVel();
    const auto safe = [](double value) {return std::isfinite(value) ? value : 0.0;};
    const double yaw = pose.Rot().Yaw();
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = rclcpp::Time(time.sec, time.nsec, RCL_ROS_TIME);
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = robot_base_frame_;
    odom.pose.pose.position.x = pose.Pos().X();
    odom.pose.pose.position.y = pose.Pos().Y();
    odom.pose.pose.position.z = pose.Pos().Z();
    odom.pose.pose.orientation.x = pose.Rot().X();
    odom.pose.pose.orientation.y = pose.Rot().Y();
    odom.pose.pose.orientation.z = pose.Rot().Z();
    odom.pose.pose.orientation.w = pose.Rot().W();
    odom.twist.twist.linear.x = safe(c * linear.X() + s * linear.Y());
    odom.twist.twist.linear.y = safe(-s * linear.X() + c * linear.Y());
    odom.twist.twist.angular.z = safe(angular.Z());
    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped tf;
    tf.header = odom.header;
    tf.child_frame_id = robot_base_frame_;
    tf.transform.translation.x = pose.Pos().X();
    tf.transform.translation.y = pose.Pos().Y();
    tf.transform.translation.z = pose.Pos().Z();
    tf.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);
  }

  GZ_REGISTER_MODEL_PLUGIN(PlanarDrivePlugin)
}
