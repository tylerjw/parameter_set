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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>

#include "robot_parameters.hpp"

namespace example {

class RobotSubsystem {
 public:
  /**
   * @brief      Example Subsystem with subscriber
   *
   * @param[in]  node              The node
   * @param[in]  robot_parameters  The robot parameters
   */
  RobotSubsystem(const std::shared_ptr<rclcpp::Node> node,
                 const RobotParameters& robot_parameters);

  /**
   * @brief      used by dynamic parameters system to update parameters
   *
   * @param[in]  robot_parameters  The robot parameters
   */
  void updateParameters(const RobotParameters& robot_parameters);

  /**
   * @brief      Gets the robot description.
   *
   * @return     The robot description.
   */
  std::string getRobotDescription() const;

 private:
  const std::shared_ptr<rclcpp::Node> node_;

  // Mutex is needed for syncornized access to RobotParamers because it can be
  // dynamically updated
  mutable std::mutex robot_parameters_mutex_;
  RobotParameters robot_parameters_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_sub_;

  // Used by constructor and dynamic parameter setting to initialize the ros
  // interfaces when they change
  void initializeRosInterfaces();

  // Joint state callback
  void jointStateCallback(
      const sensor_msgs::msg::JointState::ConstSharedPtr joint_state);
};

}  // namespace example
