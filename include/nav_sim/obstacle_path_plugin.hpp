#ifndef NAV_SIM__OBSTACLE_PATH_PLUGIN_HPP_
#define NAV_SIM__OBSTACLE_PATH_PLUGIN_HPP_

// Ported from Intent-MPC uav_simulator obstaclePathPlugin (MIT).
// Drives a Gazebo model along a configured waypoint path at a given speed,
// optionally looping or using a least-squares polynomial (sin) trajectory.
// Pure Gazebo layer: no ROS topics are published by this plugin, so it stays
// navigation-stack agnostic (any stack may consume the world as-is).

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Animation.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>

#include <Eigen/Dense>
#include <vector>

namespace gazebo
{
  class DynamicObstacle : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;

  private:
    // waypoint->pose interpolation helpers
    void linear(const double & ts,
                const std::vector<std::vector<double>> & path,
                const std::vector<double> & time,
                std::vector<std::vector<double>> & traj,
                std::vector<double> & t);
    void sine(const double & ts,
              const std::vector<std::vector<double>> & path,
              const std::vector<double> & time,
              std::vector<std::vector<double>> & traj,
              std::vector<double> & t);
    void lls(const double & ts,
             const std::vector<std::vector<double>> & path,
             const std::vector<double> & time,
             std::vector<std::vector<double>> & traj,
             std::vector<double> & t,
             const int & order);
    Eigen::MatrixXd fit(const Eigen::MatrixXd & x,
                        const Eigen::MatrixXd & y,
                        const Eigen::MatrixXd & px,
                        const int & order);
    Eigen::MatrixXd vandermonde(const Eigen::MatrixXd & x, const int & order);
    std::vector<double>& interpolateAngle(double start, double end, double dx);

    physics::ModelPtr model_;
    sdf::ElementPtr sdf_;

    double velocity_ = 1.0;
    bool orientation_ = true;
    bool loop_ = false;
    bool sinWave_ = false;
    double angularVelocity_ = 0.8;

    std::vector<ignition::math::Vector3d> path_;
    std::vector<std::vector<double>> pathWithAngle_;
    std::vector<double> timeKnot_;
  };

  GZ_REGISTER_MODEL_PLUGIN(DynamicObstacle)
}

#endif  // NAV_SIM__OBSTACLE_PATH_PLUGIN_HPP_