/*
 * Copyright (c) 2021, Rodrigo Leonello
 */

#pragma once

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "RobotArm/connection.h"
#include "RobotArm/jointLimits.h"
#include "RobotArm/robotKinematics.h"
#include "RobotArm/types.h"
#include "RobotArm/utils.h"

namespace robot_arm_control {

/**
 * This class reads information about the required trajectory of the robot manipulator from a file .in. It sends control
 * signals over the connection and receives feedback from the robot.
 */
class Control {
 public:
  explicit Control(const std::string& input);
  ~Control();

  /**
   * It sets the control settings.
   *
   * @param settings: @sa ControlSettings.
   */
  void setSettings(const ControlSettings& settings);

  /**
   * It changes the robot model settings by a given new one.
   *
   * @param settings: new settings.
   */
  void setRobotSettings(const RobotSettings& settings);

  /**
   * It creates a thread to control the arm.
   */
  void start(std::weak_ptr<Connection> connection);

  /**
   * It stops the thread which controls the arm.
   */
  void stop();

  /**
   * It gets the waypoints of the current trajectory.
   *
   * @return waypoints
   */
  std::vector<Eigen::Vector4f> getWaypoints() const;

  /**
   * It gets the current control signal.
   *
   * @return control_signal_
   */
  Eigen::Vector3f getControlSignal() const;

  /**
   * It solves the Kinematic Control problem for the 3-DoF robot arm. It computes the control signal for robot
   * kinematics.
   */
  void computeVelocityControl(const Eigen::Vector3f& joints, float t);

  std::shared_ptr<Connection> connection_;

 private:
  /**
   * It read the input file.
   *
   * @param input: absolute file path.
   */
  void readInput(const std::string& input);

  /**
   * This is the main thread of the controller. It periodically sends control signals to the robot.
   */
  void main(std::weak_ptr<Connection> connection);

  /** The robot motion can be described by: \f$\dot{q} = u\f$, where \f$u\f$
   * is the velocity control signal (vector) applied to the motor drive of each joint. \n
   * We have: \f$\dot{x} = J * \dot{q} = J * u\f$, where \f$J\f$ is the Jacobian \n
   * Then: \f$u = J^{-1} * v(t)\f$, where \f$v(t)\f$ is the cartesian control signal. \n
   * \f$e = x_s(t) - x\f$, where \f$x_s\f$ is the desired cartesian position, 
   * \f$x\f$ the current position. \n  
   * \f$\dot{e} = \dot{x_s} - v(t)\f$, then we may choose a proportional plus 
   * feedforward control law: \n
   * \f$ v(t) = \dot{x_s} + \Lambda * e \f$, where \f$L = \lambda * I\f$ 
   * (identity matrix), \f$\lambda > 0\f$ \n
   * Note that this is the Damping Method (DLS), thus if the robot starts at 
   * a position where \f$J(q0)\f$ is singular and \f$v\f$ is in its nullspace, 
   * it may be impossible to find the inverse kinematics iteratively, and the 
   * control fails. The solution is to introduce a term belonging to the 
   * nullsapce of \f$J\f$: \n
   * \f$u = J^{-1} * v + (I - J^{-1} * J) * q_0\f$, where \f$q_0\f$ is an 
   * arbitrary joint velocities to avoid singularities.
   * \param joints the current robot joints
   * \param t current time (0 means start time)
   */
  Eigen::Vector3f feedforwardControl(const Eigen::Vector3f& joints, 
    float t);

  /**
   * The Analytical Control solution uses the analytical inverse kinematics and it applies
   * \f$u = \frac{dq}{dt} = (q_s - q) / dt\f$
   *
   * @param joints: the current robot joints
   * @param t: current time (0 means start time)
   */
  Eigen::Vector3f analyticalControl(const Eigen::Vector3f& joints, float t);

  int sleep_ms_;
  RobotKinematics model_;
  Eigen::Vector3f control_signal_;

  std::thread thread_;
  std::atomic<bool> stop_;
  std::unique_ptr<Trajectory> trajectory_;
  std::chrono::time_point<std::chrono::system_clock> clock_;
};

}  // namespace robot_arm_control
