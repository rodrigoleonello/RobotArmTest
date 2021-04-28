/*
 * Copyright (c) 2021, Rodrigo Leonello
 */

#include "RobotArm/robotSystem.h"

#include <fstream>
#include <iomanip>

#include "RobotArm/utils.h"

namespace robot_arm_control {

RobotSystem::RobotSystem()
    : connection_(std::make_shared<Connection>()),
      sleep_ms_(1),
      encoder_resolution_(4096),
      control_signal_(Eigen::Vector3f::Zero()),
      stop_(false),
      clock_(std::chrono::system_clock::now()) {}

RobotSystem::~RobotSystem() { stop(); }

void RobotSystem::setSettings(const SystemSettings& settings) {
  sleep_ms_ = static_cast<int>(1000 * (1.0 / static_cast<float>(settings.frequency_)));
  encoder_resolution_ = settings.encoder_resolution_;
}

void RobotSystem::setRobotSettings(const RobotSettings& settings) { robot_.setSettings(settings); }

void RobotSystem::start(std::weak_ptr<Connection> con) {
  stop();
  stop_ = false;
  thread_ = std::thread(&RobotSystem::main, this, con);
}

void RobotSystem::stop() {
  stop_ = true;
  if (thread_.joinable()) {
    thread_.join();
  }
}

void RobotSystem::main(std::weak_ptr<Connection> con) {
  connection_->open();

  logger_ = std::make_unique<SystemLogger>("out.csv");

  while (auto conn = con.lock()) {
    if (!conn->isOpened()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } else {
      break;
    }
  }

  clock_ = std::chrono::system_clock::now();
  double elapsed_time;
  while (auto conn = con.lock()) {
    if (!conn->isOpened()) {
      break;
    }
    if (stop_) {
      break;
    }

    auto new_clock = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = new_clock - clock_;
    elapsed_time += diff.count();

    robot_.update(control_signal_, diff.count());
    auto q = robot_.getJoints();

    auto p = robot_.forwardKinematics(q);
    logger_->save(p, control_signal_, q, elapsed_time);

    mockEncoderPrecisionLost(q, encoder_resolution_);
    auto qc = eigen3fToUchar3(q);
    connection_->send(qc);

    std::vector<unsigned char> control;
    conn->receive(control);
    control_signal_ = uchar3ToEigen3f(control);
    clock_ = new_clock;
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms_));
  }
  connection_->close();
}

}  // namespace robot_arm_control
