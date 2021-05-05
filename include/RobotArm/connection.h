/*
 * Copyright (c) 2021, Rodrigo Leonello
 */

#pragma once

#include <atomic>
#include <mutex>
#include <vector>

namespace robot_arm_control {

class Connection {
 public:
  Connection() { data_.reserve(12); }
  ~Connection() {}

  void open() { opened_ = true; }
  void close() { opened_ = false; }
  bool isOpened() { return opened_; }

  /**
   * Send data to the robot (explicit pointer convertion).
   */
  void send(std::vector<unsigned char> &data) {
    std::lock_guard<std::mutex> lock(mutex_);
    data_ = data;
  }

  /**
   * Receive state of the robot (explicit pointer convertion).
   */
  void receive(std::vector<unsigned char> &data) {
    std::lock_guard<std::mutex> lock(mutex_);
    data = data_;
  }

 private:
  std::mutex mutex_;
  std::atomic<bool> opened_;
  std::vector<unsigned char> data_;
};

}  // namespace robot_arm_control
