// Copyright 2021 PickNik Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PickNik Inc. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "robot_subsystem.hpp"

namespace example {

RobotSubsystem::RobotSubsystem(const std::shared_ptr<rclcpp::Node> node,
                               const RobotParameters& robot_parameters)
    : node_{node}, robot_parameters_{robot_parameters} {
  initializeRosInterfaces();
}

void RobotSubsystem::updateParameters(const RobotParameters& robot_parameters) {
  {
    std::lock_guard<std::mutex> lock(robot_parameters_mutex_);
    robot_parameters_ = robot_parameters;
  }

  initializeRosInterfaces();
}

void RobotSubsystem::initializeRosInterfaces() {
  std::lock_guard<std::mutex> lock(robot_parameters_mutex_);
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      robot_parameters_.joint_state_topic, 25,
      std::bind(&RobotSubsystem::jointStateCallback, this,
                std::placeholders::_1));
}

void RobotSubsystem::jointStateCallback(
    const sensor_msgs::msg::JointState::ConstSharedPtr joint_state) {
  // do something with the joint state
}

std::string RobotSubsystem::getRobotDescription() const {
  std::lock_guard<std::mutex> lock(robot_parameters_mutex_);
  return robot_parameters_.robot_description;
}

}  // namespace example
