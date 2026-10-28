#include "nav_sim/obstacle_path_plugin.hpp"

#include <cmath>
#include <algorithm>
#include <utility>

namespace gazebo
{
  void DynamicObstacle::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
  {
    model_ = _parent;
    sdf_ = _sdf;

    if (sdf_->HasElement("velocity")) {
      velocity_ = sdf_->Get<double>("velocity");
    }
    if (sdf_->HasElement("orientation")) {
      orientation_ = sdf_->Get<bool>("orientation");
    }
    if (orientation_) {
      if (sdf_->HasElement("angular_velocity")) {
        angularVelocity_ = sdf_->Get<double>("angular_velocity");
      }
    }
    if (sdf_->HasElement("loop")) {
      loop_ = sdf_->Get<bool>("loop");
    }
    if (sdf_->HasElement("sin")) {
      sinWave_ = sdf_->Get<bool>("sin");
    }

    // Read waypoint path
    path_.clear();
    if (sdf_->HasElement("path")) {
      sdf::ElementPtr waypointElem = sdf_->GetElement("path")->GetElement("waypoint");
      while (waypointElem) {
        ignition::math::Vector3d wp = waypointElem->Get<ignition::math::Vector3d>();
        path_.push_back(wp);
        waypointElem = waypointElem->GetNextElement("waypoint");
      }
    }

    if (loop_) {
      path_.push_back(path_[0]);  // form a loop
    } else {
      // back and forth
      std::vector<ignition::math::Vector3d> temp = path_;
      for (int i = static_cast<int>(temp.size()) - 2; i >= 0; --i) {
        path_.push_back(temp[i]);
      }
    }

    // Build pose list (optionally with yaw following the path direction)
    pathWithAngle_.clear();
    if (orientation_) {
      double yawLast = 0.0;
      for (std::size_t i = 0; i < path_.size(); ++i) {
        const double xCurr = path_[i].X();
        const double yCurr = path_[i].Y();
        const double zCurr = path_[i].Z();

        if (i == 0) {
          const double xNext = path_[i + 1].X();
          const double yNext = path_[i + 1].Y();
          const double yawStart = std::atan2(yNext - yCurr, xNext - xCurr);
          pathWithAngle_.push_back({xCurr, yCurr, zCurr, yawStart});
          yawLast = yawStart;
        } else {
          const double xPrev = path_[i - 1].X();
          const double yPrev = path_[i - 1].Y();
          double xNext = 0.0, yNext = 0.0;
          if (i + 1 < path_.size()) {
            xNext = path_[i + 1].X();
            yNext = path_[i + 1].Y();
          } else {
            xNext = path_[1].X();
            yNext = path_[1].Y();
          }
          const double yawCurr = std::atan2(yNext - yCurr, xNext - xCurr);
          pathWithAngle_.push_back({xCurr, yCurr, zCurr, yawLast});
          pathWithAngle_.push_back({xCurr, yCurr, zCurr, yawCurr});
          yawLast = yawCurr;
          (void)xPrev; (void)yPrev;
        }
      }
    } else {
      for (const auto & wp : path_) {
        pathWithAngle_.push_back({wp.X(), wp.Y(), wp.Z(), 0.0});
      }
    }

    // Total time (forward motion + in-place rotation segments)
    timeKnot_.clear();
    double totalTime = 0.0;
    timeKnot_.push_back(totalTime);
    for (std::size_t i = 0; i < pathWithAngle_.size() - 1; ++i) {
      const auto & poseCurr = pathWithAngle_[i];
      const auto & poseNext = pathWithAngle_[i + 1];

      const bool rotation =
        (poseCurr[0] == poseNext[0]) &&
        (poseCurr[1] == poseNext[1]) &&
        (poseCurr[2] == poseNext[2]);

      if (!rotation) {
        const double dx = poseNext[0] - poseCurr[0];
        const double dy = poseNext[1] - poseCurr[1];
        const double dz = poseNext[2] - poseCurr[2];
        const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
        totalTime += distance / velocity_;
        timeKnot_.push_back(totalTime);
      } else {
        const double yawDiff = std::atan2(
          std::sin(poseNext[3] - poseCurr[3]),
          std::cos(poseNext[3] - poseCurr[3]));
        const double angleABSDiff = std::abs(yawDiff);
        totalTime += angleABSDiff / angularVelocity_;
        timeKnot_.push_back(totalTime);
      }
    }

    // Optional smooth polynomial (sine) trajectory via least-squares fit
    if (sinWave_) {
      std::vector<std::vector<double>> newTraj;
      std::vector<double> newTime;
      const double dt = 0.01;
      lls(dt, pathWithAngle_, timeKnot_, newTraj, newTime, 5);
      pathWithAngle_ = std::move(newTraj);
      timeKnot_ = std::move(newTime);
    }

    // Drive the model with a looping pose animation
    gazebo::common::PoseAnimationPtr anim(
      new gazebo::common::PoseAnimation("obstaclePathLoop", totalTime, true));
    for (std::size_t i = 0; i < pathWithAngle_.size(); ++i) {
      const double t = timeKnot_[i];
      const auto & pose = pathWithAngle_[i];
      gazebo::common::PoseKeyFrame * key = anim->CreateKeyFrame(t);
      key->Translation(ignition::math::Vector3d(pose[0], pose[1], pose[2]));
      key->Rotation(ignition::math::Quaterniond(0.0, 0.0, pose[3]));
    }
    _parent->SetAnimation(anim);
  }

  void DynamicObstacle::linear(
    const double & ts,
    const std::vector<std::vector<double>> & path,
    const std::vector<double> & time,
    std::vector<std::vector<double>> & traj,
    std::vector<double> & t)
  {
    traj.clear();
    t.clear();
    double timeStamp = time[0];
    for (std::size_t i = 0; i + 1 < path.size(); ++i) {
      const auto & currP = path[i];
      const auto & nextP = path[i + 1];
      const double timeStep = time[i + 1] - time[i];
      const int steps = std::max(1, static_cast<int>(timeStep / ts));
      for (int j = 0; j < steps; ++j) {
        std::vector<double> p(4);
        for (std::size_t k = 0; k < currP.size(); ++k) {
          p[k] = currP[k] + (nextP[k] - currP[k]) / steps * j;
        }
        traj.push_back(p);
        t.push_back(timeStamp);
        timeStamp += ts;
      }
    }
  }

  void DynamicObstacle::sine(
    const double & ts,
    const std::vector<std::vector<double>> & path,
    const std::vector<double> & time,
    std::vector<std::vector<double>> & traj,
    std::vector<double> & t)
  {
    traj.clear();
    t.clear();
    double timeStamp = time[0];
    std::vector<std::vector<double>> sineWave;
    for (std::size_t i = 0; i + 1 < path.size(); ++i) {
      const auto & currP = path[i];
      const auto & nextP = path[i + 1];
      const double timeStep = time[i + 1] - time[i];
      const int steps = std::max(1, static_cast<int>(timeStep / ts));
      sineWave.clear();
      for (int j = 0; j < steps; ++j) {
        sineWave.push_back({
          0.5 * std::sin((j * ts) * (2 * M_PI) / (currP[1] - nextP[1])), 0.0, 0.0, 0.0});
      }
      for (int j = 0; j < steps; ++j) {
        std::vector<double> p(4);
        for (std::size_t k = 0; k < currP.size(); ++k) {
          p[k] = currP[k] + (nextP[k] - currP[k]) / steps * j + sineWave[j][k];
        }
        traj.push_back(p);
        t.push_back(timeStamp);
        timeStamp += ts;
      }
    }
  }

  void DynamicObstacle::lls(
    const double & ts,
    const std::vector<std::vector<double>> & path,
    const std::vector<double> & time,
    std::vector<std::vector<double>> & traj,
    std::vector<double> & t,
    const int & order)
  {
    const double dt = ts;
    const int numberPoints = static_cast<int>(path.size());
    Eigen::MatrixXd x, y, tt;
    x.resize(numberPoints, 1);
    y.resize(numberPoints, 1);
    tt.resize(numberPoints, 1);
    for (int j = 0; j < numberPoints; ++j) {
      x(j, 0) = path[j][0];
      y(j, 0) = path[j][1];
      tt(j, 0) = time[j];
    }

    Eigen::MatrixXd X;
    X.resize(static_cast<int>(time.back() / dt), 1);
    for (int j = 0; j < static_cast<int>(time.back() / dt); ++j) {
      X(j, 0) = j * dt;
    }

    const Eigen::MatrixXd predx = fit(tt, x, X, order);
    const Eigen::MatrixXd predy = fit(tt, y, X, order);

    traj.clear();
    t.clear();
    for (int j = 0; j < predx.rows(); ++j) {
      std::vector<double> p;
      if (j + 1 < predx.rows()) {
        p = std::vector<double>{
          predx(j, 0), predy(j, 0), path[0][2],
          std::atan2(predy(j + 1, 0) - predy(j, 0), predx(j + 1, 0) - predx(j, 0))};
      } else {
        p = std::vector<double>{predx(j, 0), predy(j, 0), path[0][2], 0.0};
      }
      traj.push_back(p);
      t.push_back(j * dt);
    }
  }

  Eigen::MatrixXd DynamicObstacle::fit(
    const Eigen::MatrixXd & x,
    const Eigen::MatrixXd & y,
    const Eigen::MatrixXd & px,
    const int & order)
  {
    const Eigen::MatrixXd vx = vandermonde(x, order);
    Eigen::MatrixXd Im;
    Im.resize(order + 1, order + 1);
    Im.setIdentity();
    Im(order, order) = 0;
    const double lambda = 0.01;
    const Eigen::MatrixXd Xt = vx.transpose();
    const Eigen::MatrixXd XtX = Xt * vx + lambda * Im;
    const Eigen::MatrixXd XtXinverse = XtX.inverse();
    const Eigen::MatrixXd Xty = Xt * y;
    const Eigen::MatrixXd w = XtXinverse * Xty;
    const Eigen::MatrixXd X = vandermonde(px, order);
    return X * w;
  }

  Eigen::MatrixXd DynamicObstacle::vandermonde(const Eigen::MatrixXd & x, const int & order)
  {
    Eigen::MatrixXd X;
    X.resize(x.rows(), order + 1);
    for (int j = 0; j < x.rows(); ++j) {
      for (int k = 0; k < order + 1; ++k) {
        X(j, k) = std::pow(x(j, 0), k);
      }
    }
    return X;
  }

  std::vector<double>& DynamicObstacle::interpolateAngle(double start, double end, double dx)
  {
    static std::vector<double> interpolation;
    interpolation.clear();
    const double angleDiff = end - start;
    const double angleDiffABS = std::abs(angleDiff);

    if (angleDiff >= 0 && angleDiffABS <= M_PI) {
      for (double mid = start + dx; mid < end; mid += dx) {
        interpolation.push_back(mid);
      }
    } else if (angleDiff >= 0 && angleDiffABS > M_PI) {
      for (double mid = start - dx; mid >= -M_PI; mid -= dx) {
        interpolation.push_back(mid);
      }
      for (double mid = M_PI; mid > end; mid -= dx) {
        interpolation.push_back(mid);
      }
    } else if (angleDiff < 0 && angleDiffABS <= M_PI) {
      for (double mid = start - dx; mid > end; mid -= dx) {
        interpolation.push_back(mid);
      }
    } else {
      for (double mid = start + dx; mid <= M_PI; mid += dx) {
        interpolation.push_back(mid);
      }
      for (double mid = -M_PI; mid < end; mid += dx) {
        interpolation.push_back(mid);
      }
    }
    return interpolation;
  }
}