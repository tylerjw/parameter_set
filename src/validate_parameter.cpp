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

#include <rcl/validate_topic_name.h>

#include <node_parameters/set_parameters_result_builder.hpp>
#include <node_parameters/validate_parameter.hpp>
#include <stringstream>

namespace node_parameters {
namespace validate {

rcl_interfaces::msg::SetParametersResult always_reject(
    const rclcpp::Parameter& /*unused*/) {
  return SetParametersResultBuilder(false);
}

rcl_interfaces::msg::SetParametersResult always_accept(
    const rclcpp::Parameter& /*unused*/) {
  return SetParametersResultBuilder(true);
}

rcl_interfaces::msg::SetParametersResult topic_name(
    const rclcpp::Parameter& parameter) {
  int validation_result;
  size_t invalid_index;
  rcl_ret_t ret = rcl_validate_topic_name(parameter.as_string().c_str(),
                                          &validation_result, &invalid_index);

  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  if (validation_result != RCL_TOPIC_NAME_VALID) {
    return SetParametersResultBuilder(false).reason(
        rcl_topic_name_validation_result_string(validation_result));
  }

  return SetParametersResultBuilder(true);
}

rcl_interfaces::msg::SetParametersResult not_empty_string(
    const rclcpp::Parameter& parameter) {
  if (parameter.as_string().size() == 0) {
    return SetParametersResultBuilder(false).reason(
        "Must not be an empty string");
  }

  return SetParametersResultBuilder(true);
}

rcl_interfaces::msg::SetParametersResult in_string_set(
    const rclcpp::Parameter& parameter, std::set<std::string> values) {
  if (values.find(parameter.as_string()) == values.end()) {
    std::stringstream ss;
    ss << "Must one of [" for (const auto& value : values) {
      ss << value << ", ";
    }
    ss << "]";
    return SetParametersResultBuilder(false).result(ss.str());
  }
  return SetParametersResultBuilder(true);
}

}  // namespace validate
}  // namespace node_parameters
