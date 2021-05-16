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

#include "parameters.hpp"

#include <node_parameters/node_parameters.hpp>

using node_parameters::ParameterDescriptorBuilder;
using node_parameters::SetParametersResultBuilder;
using rcl_interfaces::msg::ParameterType;

namespace example {

bool RobotParameters::declare(node_parameters::NodeParameters* node_parameters,
                              std::shared_ptr<rclcpp::Node> node) {
  auto ns = getNamespace();
  node->declare_parameter(
      ns + ".robot_description", robot_description,
      ParameterDescriptorBuilder()
          .type(ParameterType::PARAMETER_STRING)
          .description("Parameter containing XML robotic description"));

  node->declare_parameter(
      ns + ".joint_state_topic", joint_state_topic,
      ParameterDescriptorBuilder()
          .type(ParameterType::PARAMETER_STRING)
          .description("Topic to subscribe to for joint states"));

  return true;
}

bool RobotParameters::get(std::shared_ptr<rclcpp::Node> node) {
  auto ns = getNamespace();
  bool success = true;
  success &= node->get_parameter(ns + ".robot_description", robot_description);
  success &= node->get_parameter(ns + ".joint_state_topic", joint_state_topic);

  return success;
}

bool PlanningParameters::declare(
    node_parameters::NodeParameters* node_parameters,
    std::shared_ptr<rclcpp::Node> node) {
  auto ns = getNamespace();
  node->declare_parameter(
      ns + ".pipeline_names", pipeline_names,
      ParameterDescriptorBuilder()
          .type(ParameterType::PARAMETER_STRING_ARRAY)
          .description("Planning Pipelines to load")
          .additional_constraints("must specify alteast one"));
  node_parameters->registerValidateFunction(
      ns + ".pipeline_names", [](const rclcpp::Parameter& parameter) {
        if (parameter.as_string_array().size() < 1) {
          return SetParametersResultBuilder(false).reason(
              "Must specify atleast one Planning Pipeline");
        } else {
          return SetParametersResultBuilder(true);
        }
      });

  node->declare_parameter(
      ns + ".planning_attempts", planning_attempts,
      ParameterDescriptorBuilder()
          .type(ParameterType::PARAMETER_INTEGER)
          .description("Number of times to attempt planning")
          .integer_range(1));

  node->declare_parameter(ns + ".max_velocity_scaling_factor",
                          max_velocity_scaling_factor,
                          ParameterDescriptorBuilder()
                              .type(ParameterType::PARAMETER_DOUBLE)
                              .description("Max velocity scaling factor")
                              .floating_point_range(0.0, 1.0));

  node->declare_parameter(ns + ".max_acceleration_scaling_factor",
                          max_acceleration_scaling_factor,
                          ParameterDescriptorBuilder()
                              .type(ParameterType::PARAMETER_DOUBLE)
                              .description("Max acceleration scaling factor")
                              .floating_point_range(0.0, 1.0));

  return true;
}

bool PlanningParameters::get(std::shared_ptr<rclcpp::Node> node) {
  auto ns = getNamespace();
  bool success = true;
  success &= node->get_parameter(ns + ".pipeline_names", pipeline_names);
  success &= node->get_parameter(ns + ".planning_attempts", planning_attempts);
  success &= node->get_parameter(ns + ".max_velocity_scaling_factor",
                                 max_velocity_scaling_factor);
  success &= node->get_parameter(ns + ".max_acceleration_scaling_factor",
                                 max_acceleration_scaling_factor);

  return success;
}

}  // namespace example
