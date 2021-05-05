/*
 * Copyright (c) 2021, Rodrigo Leonello
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

namespace robot_arm_control {

/**
 * Class to handle joint angles limits.
 */
template <class T = float>
class JointLimits {
 public:
  JointLimits(T value = T{}, T min = static_cast<T>(-M_PI), T max = static_cast<T>(M_PI))
      : value_(value), min_(min), max_(max) {
    norm();
  }

  T operator()(void) { return value_; }

  T operator()(const T& new_value) {
    value_ = new_value;
    norm();
    return value_;
  }

  JointLimits& operator+=(const JointLimits& rhs) {
    value_ += rhs.value_;
    norm();
    return *this;
  }

  friend JointLimits operator+(JointLimits lhs, const JointLimits& rhs) {
    lhs.value += rhs.value;
    norm();
    return lhs;
  }

  JointLimits& operator=(const JointLimits& other) {
    value_ = other.value_;
    min_ = other.min_;
    max_ = other.max_;
  }

  JointLimits& operator=(const T& other) {
    value_ = other;
    norm();
  }

 private:
  void norm() {
    value_ = fmod(value_, 2 * M_PI);
    value_ += (value_ > M_PI) ? -2 * M_PI : 0;
    value_ += (value_ < -M_PI) ? 2 * M_PI : 0;
    value_ = std::min(std::max(value_, min_), max_);
  }

  T value_;
  T min_;
  T max_;
};

}  // namespace robot_arm_control
