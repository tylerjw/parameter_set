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

#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <string>

namespace parameter_set {
namespace validate {

/**
 * @breif Function interface for parameter validation
 */
using ValidateFunction = std::function<rcl_interfaces::msg::SetParametersResult(
    const rclcpp::Parameter&)>;

/**
 * @brief Parameter is not dynamic
 *
 * @return     results is always false
 */
rcl_interfaces::msg::SetParametersResult always_reject(
    const rclcpp::Parameter& /*unused*/);

/**
 * @brief Parameter is dynamic
 *
 * @return     results is always true
 */
rcl_interfaces::msg::SetParametersResult always_accept(
    const rclcpp::Parameter& /*unused*/);

/**
 * @brief      Validate a ROS topic name
 *
 * @param[in]  parameter The parameter
 *
 * @return     Success or Failure with reason
 */
rcl_interfaces::msg::SetParametersResult topic_name(
    const rclcpp::Parameter& parameter);

/**
 * @brief      Validate string is not empty
 *
 * @param[in]  parameter  The parameter
 *
 * @return     Success or Failure with reason
 */
rcl_interfaces::msg::SetParametersResult not_empty_string(
    const rclcpp::Parameter& parameter);

/**
 * @brief      Validate that string is in set of strings
 *
 * @param[in]  parameter  The parameter
 * @param[in]  values     The set of strings
 *
 * @return     Success or Failure with reason
 */
rcl_interfaces::msg::SetParametersResult in_string_set(
    const rclcpp::Parameter& parameter, std::set<std::string> values);

}  // namespace validate
}  // namespace parameter_set
