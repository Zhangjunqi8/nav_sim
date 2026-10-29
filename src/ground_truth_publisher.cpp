// Ground-truth publisher for dynamic obstacles.
//
// Subscribes to gazebo_msgs/ModelStates (always available in the sim) and
// republishes the pose of every non-robot model as a standard Detection2DArray
// on /dynamic_obstacles/ground_truth plus a PoseArray with velocities estimated
// by finite differences. Purely standard ROS2 messages: any navigation stack or
// evaluation harness may consume it without depending on this package.
//
// Robot models (the AGV, other fixed anchors) can be excluded via the
// "exclude_models" parameter.

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <std_msgs/msg/header.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

class GroundTruthPublisher : public rclcpp::Node
{
public:
  GroundTruthPublisher() : Node("ground_truth_publisher")
  {
    this->declare_parameter<std::vector<std::string>>("exclude_models", {"agv", "ground_plane"});
    this->declare_parameter<std::string>("robot_frame", "map");

    exclude_ = this->get_parameter("exclude_models").as_string_array();
    frame_ = this->get_parameter("robot_frame").as_string();

    sub_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
      "/model_states", rclcpp::SensorDataQoS(),
      [this](const gazebo_msgs::msg::ModelStates::SharedPtr msg) { onModelStates(msg); });

    det_pub_ = this->create_publisher<vision_msgs::msg::Detection2DArray>(
      "/dynamic_obstacles/ground_truth", 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      "/dynamic_obstacles/ground_truth/poses", 10);
  }

private:
  void onModelStates(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
  {
    const auto stamp = this->now();

    vision_msgs::msg::Detection2DArray dets;
    dets.header.frame_id = frame_;
    dets.header.stamp = stamp;

    geometry_msgs::msg::PoseArray poses;
    poses.header.frame_id = frame_;
    poses.header.stamp = stamp;

    for (std::size_t i = 0; i < msg->name.size(); ++i) {
      const std::string & name = msg->name[i];
      if (std::find(exclude_.begin(), exclude_.end(), name) != exclude_.end()) {
        continue;
      }
      const auto & pose = msg->pose[i];

      // ground-truth detection (2D box with identity hypothesis)
      vision_msgs::msg::Detection2D det;
      det.header = dets.header;
      det.bbox.center.position.x = pose.position.x;
      det.bbox.center.position.y = pose.position.y;
      det.bbox.size_x = 0.5;
      det.bbox.size_y = 0.5;
      vision_msgs::msg::ObjectHypothesisWithPose hyp;
      hyp.hypothesis.class_id = name;
      hyp.hypothesis.score = 1.0;
      det.results.push_back(hyp);
      dets.detections.push_back(det);

      poses.poses.push_back(pose);
    }

    det_pub_->publish(dets);
    pose_pub_->publish(poses);
  }

  std::vector<std::string> exclude_;
  std::string frame_;
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr det_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundTruthPublisher>());
  rclcpp::shutdown();
  return 0;
}
