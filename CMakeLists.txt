cmake_minimum_required(VERSION 3.1)
project(RobotArm)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(
  include
)

add_executable(${PROJECT_NAME} 
  src/controller.cpp
  src/main.cpp
  src/robotKinematics.cpp
  src/robotSystem.cpp
  src/utils.cpp
)

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen -pthread)
