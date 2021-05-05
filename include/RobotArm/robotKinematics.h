/*
 * Copyright (c) 2021, Rodrigo Leonello
 */

#pragma once

#include <Eigen/Geometry>
#include <functional>
#include <memory>
#include <vector>

#include "RobotArm/jointLimits.h"
#include "RobotArm/types.h"

namespace robot_arm_control {

/**
 * The Class Robot calculates the forward and inverse kinematics of a robot arm.
 *
 */
class RobotKinematics {
 public:
  RobotKinematics();
  RobotKinematics(float q1, float q2, float q3);
  ~RobotKinematics() = default;

  /**
   * It sets the robot settings.
   *
   * @param settings @sa RobotSettings.
   */
  void setSettings(const RobotSettings& settings);

  /**
   * The forward kinematics is computed by the multiplication of the
   * homogeneous transformations \f$ T_0*T_1*T_2*T_3 \f$, which can be formed
   * directly from the DH parameters. 
   *
   * @param joints the current values for robot joints
   */
  Eigen::Vector3f forwardKinematics(const Eigen::Vector3f& joints);

  /** It computes the analytical solution for inverse kinematics, for this 
   * specific case by algebric manipulation of the translation vector, and using 
   * trigonometric equalities. Note that we may have more than one solution
   * for some positions.
   * \param x 
   * \param y
   * \param z 
   * 
   * \return the joints to reach (x, y, z)
  */
  Eigen::Vector3f inverseKinematics(float x, float y, float z,
                                    const Eigen::Vector3f& q0 = {0, 0, 0},
                                    float error = 1e-3);

  /**
   * It returns the robot joints. This is not the one that is sent (no connection here).
   *
   * @return robot's joints.
   */
  Eigen::Vector3f getJoints();

  /**
   * It is the system model \f$ \dot{q}=u \f$, thus \f$ q=u*dt \f$. It integrates the input control.
   *
   * @param u the input control.
   * @param dt the integration step.
   */
  void update(const Eigen::Vector3f& u, float dt);

  /**
   * It calculates the Jacobian (\f$\frac{dP}{d\theta}\f$ only translation).
   *
   * @param joints the current values for robot joints.
   *
   * @return the jacobian as an eigen matrix (3x3).
   */
  JacobMP jacob(const Eigen::Vector3f& joints);

 private:

  std::unique_ptr<RobotJoints> joints_;
};

}  // namespace robot_arm_control
