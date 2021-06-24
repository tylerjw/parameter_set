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

#include <rclcpp/rclcpp.hpp>

#include "robot_parameters.hpp"
#include "robot_subsystem.hpp"

using example::RobotParameters;
using example::RobotSubsystem;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.use_intra_process_comms(true);
  auto node = std::make_shared<rclcpp::Node>("example_node", node_options);
  auto executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();

  // Declare and get parameters for the node
  node_parameters::NodeParameters node_parameters(node);
  auto robot_parameters =
      node_parameters.declare_and_get<RobotParameters>("robot");

  // Create robot subsystem and register callback for dynamic parameters
  RobotSubsystem robot_subsystem(node, robot_parameters);
  node_parameters.registerSetChangedCallback("robot", [&]() {
    robot_subsystem.updateParameters(
        node_parameters.get<RobotParameters>(robot_parameters.getNamespace()));
  });

  executor->add_node(node);
  executor->spin();
  rclcpp::shutdown();
  return 0;
}
