/*
 * Copyright (c) 2021, Rodrigo Leonello
 */

#pragma once

#include <atomic>
#include <memory>
#include <thread>

#include "RobotArm/connection.h"
#include "RobotArm/robotKinematics.h"
#include "RobotArm/systemLogger.h"

namespace robot_arm_control {

class RobotSystem {
 public:
  RobotSystem();
  ~RobotSystem();

  /**
   * It sets the system settings.
   *
   * @param settings @sa SystemSettings.
   */
  void setSettings(const SystemSettings& settings);

  /**
   * It changes the robot model settings by a given new one.
   *
   * @param settings new settings
   */
  void setRobotSettings(const RobotSettings& settings);

  /**
   * It creates a thread to start the robotic system.
   */
  void start(std::weak_ptr<Connection> con);

  /**
   * It stops the main thread which exchange information with the controller.
   */
  void stop();

  std::shared_ptr<Connection> connection_;

 private:
  /**
   * The main thread sends encoder's outputs and it gets control signals.
   */
  void main(std::weak_ptr<Connection> connection);

  int sleep_ms_;
  int encoder_resolution_;
  RobotKinematics robot_;
  Eigen::Vector3f control_signal_;

  std::thread thread_;
  std::atomic<bool> stop_;
  std::unique_ptr<SystemLogger> logger_;
  std::chrono::time_point<std::chrono::system_clock> clock_;
};

}  // namespace robot_arm_control
