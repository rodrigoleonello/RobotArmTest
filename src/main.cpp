/*
 * Copyright (c) 2021, Rodrigo Leonello
 */

#include <fstream>

#include "3rdparty/json.hpp"
#include "RobotArm/controller.h"
#include "RobotArm/robotSystem.h"
#include "RobotArm/utils.h"

int main(int argc, char **argv) {
  assert(argc == 3);

  robot_arm_control::Control control(argv[1]);
  robot_arm_control::RobotSystem system;

  std::ifstream i(argv[2]);
  nlohmann::json j;
  i >> j;

  control.setSettings(robot_arm_control::parseControlSetting(j));
  control.setRobotSettings(robot_arm_control::parseRobotSetting(j));

  system.setSettings(robot_arm_control::parseSystemSetting(j));
  system.setRobotSettings(robot_arm_control::parseRobotSetting(j));

  system.start(control.connection_);
  control.start(system.connection_);
  std::this_thread::sleep_for(std::chrono::milliseconds(11000));
}
