/*
 * Copyright (c) 2021, Rodrigo Leonello
 */

#pragma once

#include <Eigen/Geometry>
#include <algorithm>
#include <memory>
#include <vector>

#include "RobotArm/jointLimits.h"
#include "RobotArm/trajectory.h"

namespace robot_arm_control {

typedef Eigen::Matrix<float, 6, 3> JacobM;
typedef Eigen::Matrix<float, 3, 3> JacobMP;
typedef JointLimits<float> JointLimited;

struct RobotJoints {
  JointLimited q1_;
  JointLimited q2_;
  JointLimited q3_;
  RobotJoints(float q1mM[2], float q2mM[2], float q3mM[2]) {
    q1_ = JointLimited(0, q1mM[0], q1mM[1]);
    q2_ = JointLimited(0, q2mM[0], q2mM[1]);
    q3_ = JointLimited(0, q3mM[0], q3mM[1]);
  }
};

/**
 * Robot Settings
 */
struct RobotSettings {
  float joints_min_[3];
  float joints_max_[3];
  RobotSettings() : joints_min_{-M_PI, -M_PI_2, -M_PI}, joints_max_{M_PI, M_PI_2, M_PI} {}
};

/**
 * Control Settings
 */
struct ControlSettings {
  int frequency_;
  ControlSettings() : frequency_(50) {}
};

/**
 * System Settings
 */
struct SystemSettings {
  int frequency_;
  int encoder_resolution_;
  SystemSettings() : frequency_(50), encoder_resolution_(4096) {}
};

}  // namespace robot_arm_control
