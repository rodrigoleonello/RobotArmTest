/*
 * Copyright (c) 2021, Rodrigo Leonello
 */

#include "RobotArm/robotKinematics.h"

#include "RobotArm/utils.h"

namespace robot_arm_control {

RobotKinematics::RobotKinematics() {
  float q1mM[2] = {-M_PI, M_PI};
  float q2mM[2] = {-M_PI_2, M_PI_2};
  float q3mM[2] = {-M_PI, M_PI};
  joints_ = std::make_unique<RobotJoints>(q1mM, q2mM, q3mM);
}

RobotKinematics::RobotKinematics(float q1, float q2, float q3) : RobotKinematics() {
  joints_->q1_(q1);
  joints_->q1_(q2);
  joints_->q1_(q3);
}

void RobotKinematics::setSettings(const RobotSettings& settings) {
  float q1mM[2] = {settings.joints_min_[0], settings.joints_max_[0]};
  float q2mM[2] = {settings.joints_min_[1], settings.joints_max_[1]};
  float q3mM[2] = {settings.joints_min_[2], settings.joints_max_[2]};
  joints_ = std::make_unique<RobotJoints>(q1mM, q2mM, q3mM);
}

Eigen::Vector3f RobotKinematics::forwardKinematics(const Eigen::Vector3f& joints) {
  auto T1 = DHToAffine(joints[0], M_PI_2, 10, 0);
  auto T2 = DHToAffine(joints[1], 0, 5, 0);
  auto T3 = DHToAffine(joints[2], 0, 5, 0);
  Eigen::Vector3f xyz = (T1 * T2 * T3).translation();
  return xyz;
}

JacobMP RobotKinematics::jacob(const Eigen::Vector3f& joints) {
  float s1 = sin(joints[0]);
  float s2 = sin(joints[1]);
  float c1 = cos(joints[0]);
  float c2 = cos(joints[1]);
  float c23 = cos(joints[1] + joints[2]);
  float s23 = sin(joints[1] + joints[2]);

  JacobMP m;
  m << -5 * s1 * (c23 + c2 + 2), -5 * c1 * (s23 + s2), -5 * s23 * c1, 5 * c1 * (c23 + c2 + 2), -5 * s1 * (s23 + s2),
      -5 * s23 * s1, 0, 5 * (c23 + c2), 5 * c23;
  return m;
}

Eigen::Vector3f RobotKinematics::inverseKinematics(float x, float y, float z, const Eigen::Vector3f& q0, float error) {
  
  Eigen::Vector3f q;
  q[0] = atan2(y, x);
  
  float c1 = cos(q[0]);
  float zs = z * z;
  float c1s = c1 * c1;
  float xs = x * x;
  
  if (abs(c1s) <= 1e-5) { // x ~= 0
    q[2] = acos(0.5 * (2 + (zs / (25))));
    float c3 = cos(q[2]);
    float s3 = sin(q[2]);
    q[1] =  asin((1 / (2 + 2 * c3)) * (z * (1 + c3) / 5 + 2 * s3));
  }
  else {
    q[2] = acos(0.5 * (2 + (1 / (25 * c1s)) * (xs + zs * c1s) - 4 * x / (5 * c1)));
    float c3 = cos(q[2]);
    float s3 = sin(q[2]);
    q[1] = asin((1 / (2 + 2 * c3)) * (z * (1 + c3) / 5 - x * s3 / (5 * c1) + 
      2 * s3));
  }
  return q;
}

Eigen::Vector3f RobotKinematics::getJoints() {
  Eigen::Vector3f joints;
  joints[0] = joints_->q1_();
  joints[1] = joints_->q2_();
  joints[2] = joints_->q3_();
  return joints;
}

void RobotKinematics::update(const Eigen::Vector3f& u, float dt) {
  joints_->q1_ += u[0] * dt;
  joints_->q2_ += u[1] * dt;
  joints_->q3_ += u[2] * dt;
}

}  // namespace robot_arm_control
