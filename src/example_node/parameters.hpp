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

#include <node_parameters/node_parameters.hpp>
#include <string>
#include <vector>

using node_parameters::ParameterSet;

namespace example {

class RobotParameters : public ParameterSet {
 public:
  using ParameterSet::ParameterSet;

  // parameters with default values
  std::string robot_description = "robot_description";
  std::string joint_state_topic = "/joint_states";

  bool declare(node_parameters::NodeParameters* node_parameters,
               std::shared_ptr<rclcpp::Node> node) override;
  bool get(std::shared_ptr<rclcpp::Node> node) override;
};

class PlanningParameters : public ParameterSet {
 public:
  using ParameterSet::ParameterSet;

  // parameters with default values
  std::vector<std::string> pipeline_names = {"ompl"};
  int64_t planning_attempts = 10;
  double max_velocity_scaling_factor = 1.0;
  double max_acceleration_scaling_factor = 1.0;

  bool declare(node_parameters::NodeParameters* node_parameters,
               std::shared_ptr<rclcpp::Node> node) override;
  bool get(std::shared_ptr<rclcpp::Node> node) override;
};

}  // namespace example
