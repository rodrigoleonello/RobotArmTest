/*
 * Copyright (c) 2021, Rodrigo Leonello
 */

#pragma once

#include <Eigen/Geometry>
#include <algorithm>
#include <vector>

/**
 * This is a very simple way to make a piecewise-linear trajectory that computes position and velocity (constant) given
 * time and waypoints.
 *
 */
class Trajectory {
 public:
  explicit Trajectory(const std::vector<Eigen::Vector4f>& waypoints) : waypoints_(waypoints) {
    times_.reserve(waypoints_.size());
    for (auto waypoint : waypoints_) {
      times_.push_back(waypoint[3]);
    }
  }

  bool update(float t) {
    auto it = std::lower_bound(times_.begin(), times_.end(), t);
    if (it == times_.end()) {
      return false;
    }

    size_t index = it - times_.begin();
    index = (index >= times_.size()) ? times_.size() - 1 : index;
    auto t0 = (index == 0) ? 0 : times_[index - 1];
    auto tf = times_[index];
    auto x0 = (index == 0) ? Eigen::Vector3f::Zero() : Eigen::Vector3f(waypoints_[index - 1].head(3));
    auto xf = Eigen::Vector3f(waypoints_[index].head(3));

    // x(t) = x0 + v(t-t0)
    // v = (xf - x0) / (tf - t0)
    if (tf == t0) {
      v_ = Eigen::Vector3f::Zero();
      x_ = xf;
      return true;
    }
    v_ = (xf - x0) / (tf - t0);
    x_ = x0 + v_ * (t - t0);
    return true;
  }

  Eigen::Vector3f x_;
  Eigen::Vector3f v_;
  std::vector<Eigen::Vector4f> waypoints_;
  std::vector<float> times_;
};
