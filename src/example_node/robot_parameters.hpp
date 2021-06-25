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

#include <parameter_set/parameter_set.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

using parameter_set::ParameterSet;
using parameter_set::ParameterSetFactory;
using rclcpp::node_interfaces::NodeParametersInterface;

namespace example {

struct RobotParameters : public ParameterSet {
  using ParameterSet::ParameterSet;

  // parameters with default values
  std::string robot_description =
      "robot_description";  // config for reading robot_description
  std::string joint_state_topic =
      "/joint_states";  // topic for subscribing to joint states

  /**
   * @brief      Declare the parameters, called by ParameterSetFactory
   *
   * @param[in]  parameter_set_factory  The parameter set factory pointer
   * @param[in]  node_parameters        The node parameters interface
   *
   * @return     true on success
   */
  bool declare(
      ParameterSetFactory* parameter_set_factory,
      const NodeParametersInterface::SharedPtr& node_parameters) override;

  /**
   * @brief      Get the parameters, called by ParameterSetFactory
   *
   * @param[in]  node_parameters        The node parameters interface
   *
   * @return     true on success
   */
  bool get(const NodeParametersInterface::SharedPtr& node_parameters) override;
};

}  // namespace example
