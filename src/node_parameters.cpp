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

#include <iostream>
#include <node_parameters/node_parameters.hpp>
#include <utility>

namespace node_parameters {

NodeParameters::NodeParameters(std::shared_ptr<rclcpp::Node> node)
    : node_(node),
      on_set_callback_handle_{node->add_on_set_parameters_callback(
          std::bind(&NodeParameters::setParametersCallback, this,
                    std::placeholders::_1))} {}

NodeParameters::~NodeParameters() {
  if (set_changed_callback_thread_.joinable()) {
    set_changed_callback_thread_.join();
  }
}

void NodeParameters::registerValidateFunction(std::string name,
                                              ValidateFunction validate) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  validate_functions_[name].push_back(validate);
}

void NodeParameters::registerSetChangedCallback(
    std::string name, std::function<void()> callback) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  set_changed_callbacks_[name] = callback;
}

rcl_interfaces::msg::SetParametersResult NodeParameters::setParametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
  const std::lock_guard<std::recursive_mutex> guard(mutex_);

  std::set<std::string> changed_set_namespaces;

  // First validate
  for (const auto& parameter : parameters) {
    auto result = validateParameter(parameter);
    if (!result.successful) {
      return result;
    }

    // Get the set namespaces that have a parameter change
    auto split_name = split_parameter_name(parameter.get_name());
    changed_set_namespaces.insert(split_name.first);
  }

  // Populate the callbacks we are going to call on another thread
  std::vector<std::function<void()> > callbacks;
  for (const auto& set_namespace : changed_set_namespaces) {
    const std::lock_guard<std::recursive_mutex> guard(mutex_);
    auto search = set_changed_callbacks_.find(set_namespace);
    if (search != set_changed_callbacks_.end()) {
      callbacks.push_back(search->second);
    }
  }

  if (callbacks.size() > 0) {
    // If the thread for callbacks is joinable, join it
    if (set_changed_callback_thread_.joinable()) {
      set_changed_callback_thread_.join();
    }

    // Use a separte theread to call the callbacks so they can get the
    // parameters that changed within the callback.
    set_changed_callback_thread_ = std::thread([callbacks]() {
      for (const auto& callback : callbacks) {
        callback();
      }
    });
  }

  return SetParametersResultBuilder(true);
}

rcl_interfaces::msg::SetParametersResult NodeParameters::validateParameter(
    const rclcpp::Parameter& parameter) const {
  const std::lock_guard<std::recursive_mutex> guard(mutex_);
  rcl_interfaces::msg::SetParametersResult result =
      SetParametersResultBuilder(true);
  auto search = validate_functions_.find(parameter.get_name());
  if (search != validate_functions_.end()) {
    for (const auto& validate_function : search->second) {
      try {
        result = validate_function(parameter);
      } catch (const rclcpp::ParameterTypeException& exeption) {
        result = SetParametersResultBuilder(false).reason(exeption.what());
      }
      if (!result.successful) {
        return result;
      }
    }
  }

  return SetParametersResultBuilder(true);
}

std::pair<std::string, std::string> split_parameter_name(
    const std::string& full_name) {
  std::string delimiter = ".";
  auto pos = full_name.rfind(delimiter);

  if (pos != std::string::npos) {
    return std::make_pair(
        full_name.substr(0, pos),
        full_name.substr(pos + delimiter.length(), full_name.length() - pos));
  } else {
    return std::make_pair("", full_name);
  }
}

}  // namespace node_parameters
