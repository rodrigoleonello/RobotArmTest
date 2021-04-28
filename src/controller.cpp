/*
 * Copyright (c) 2021, Rodrigo Leonello
 */

#include "RobotArm/controller.h"

#include <fstream>
#include <sstream>

namespace robot_arm_control {

Control::Control(const std::string& input)
    : connection_(std::make_shared<Connection>()),
      sleep_ms_(20),
      stop_(false),
      clock_(std::chrono::system_clock::now()) {
  readInput(input);
}

Control::~Control() { stop(); }

void Control::setSettings(const ControlSettings& settings) {
  sleep_ms_ = std::max(20, static_cast<int>(1000 * (1.0 / static_cast<float>(settings.frequency_))));
}

void Control::setRobotSettings(const RobotSettings& settings) { model_.setSettings(settings); }

void Control::start(std::weak_ptr<Connection> con) {
  stop();
  stop_ = false;
  thread_ = std::thread(&Control::main, this, con);
}

void Control::stop() {
  stop_ = true;
  if (thread_.joinable()) {
    thread_.join();
  }
}

void Control::readInput(const std::string& input) {
  std::vector<Eigen::Vector4f> waypoints;
  std::ifstream file(input);
  std::string line;
  while (std::getline(file, line)) {
    std::istringstream iss(line);
    float x, y, z, t;
    if (!(iss >> x >> y >> z >> t)) {
      break;
    }
    waypoints.push_back(Eigen::Vector4f(x, y, z, t));
  }
  trajectory_ = std::make_unique<Trajectory>(std::move(waypoints));
}

void Control::main(std::weak_ptr<Connection> con) {
  connection_->open();
  while (auto conn = con.lock()) {
    if (!conn->isOpened()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } else {
      break;
    }
  }

  clock_ = std::chrono::system_clock::now();
  while (auto conn = con.lock()) {
    if (!conn->isOpened()) {
      break;
    }
    if (stop_) {
      break;
    }
    std::vector<unsigned char> joints;
    conn->receive(joints);
    auto q = uchar3ToEigen3f(joints);

    auto new_clock = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = new_clock - clock_;
    computeVelocityControl(q, diff.count());
    auto u_eigen = getControlSignal();
    auto u = eigen3fToUchar3(u_eigen);
    connection_->send(u);
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms_));
  }
  connection_->close();
}

std::vector<Eigen::Vector4f> Control::getWaypoints() const { return trajectory_->waypoints_; }

Eigen::Vector3f Control::getControlSignal() const { return control_signal_; }

void Control::computeVelocityControl(const Eigen::Vector3f& joints, float t) {
  control_signal_ = feedforwardControl(joints, t);
}

Eigen::Vector3f Control::feedforwardControl(const Eigen::Vector3f& q,
    float t) {
  auto q0dot = Eigen::Vector3f(5, 5, 5);
  auto x = model_.forwardKinematics(q);
  if (!trajectory_->update(t)) {
    return Eigen::Vector3f::Zero();
  }
  auto dx = trajectory_->x_ - x;
  auto v = trajectory_->v_ + Eigen::Matrix3f::Identity() * dx;
  auto J = model_.jacob(q);
  auto Jt = J.transpose();
  auto JJt = J * Jt;
  auto w = JJt.determinant();
  auto alpha = (w >= 0.001) ? 0 : 0.01 * (1 - w / 0.001) * (1 - w / 0.001);
  auto L = Eigen::Matrix3f::Identity() * alpha;
  auto Ji = Jt * (JJt + L).inverse();
  return Ji * v + (Eigen::Matrix3f::Identity() - Ji * J) * q0dot;
}

}  // namespace robot_arm_control
