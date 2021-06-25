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

#include "robot_parameters.hpp"

#include <parameter_set/parameter_set.hpp>
#include <parameter_set/validate_parameter.hpp>
#include <rclcpp/rclcpp.hpp>

using parameter_set::ParameterDescriptorBuilder;
using rcl_interfaces::msg::ParameterType;
using rclcpp::ParameterValue;

namespace example {

bool RobotParameters::declare(
    ParameterSetFactory* parameter_set_factory,
    const NodeParametersInterface::SharedPtr& node_parameters) {
  auto ns = getNamespace();
  node_parameters->declare_parameter(
      ns + ".robot_description", ParameterValue(robot_description),
      ParameterDescriptorBuilder()
          .type(ParameterType::PARAMETER_STRING)
          .description("Parameter containing XML robotic description"));

  node_parameters->declare_parameter(
      ns + ".joint_state_topic", ParameterValue(joint_state_topic),
      ParameterDescriptorBuilder()
          .type(ParameterType::PARAMETER_STRING)
          .description("Topic to subscribe to for joint states")
          .additional_constraints("Must be a valid topic name"));
  parameter_set_factory->registerValidateFunction(
      ns + ".joint_state_topic", &parameter_set::validate::topic_name);

  return true;
}

bool RobotParameters::get(
    const NodeParametersInterface::SharedPtr& node_parameters) {
  auto ns = getNamespace();
  robot_description =
      node_parameters->get_parameter(ns + ".robot_description").as_string();
  joint_state_topic =
      node_parameters->get_parameter(ns + ".joint_state_topic").as_string();

  return true;
}

}  // namespace example
