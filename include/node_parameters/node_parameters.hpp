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
#include <map>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <type_traits>

#include "node_parameters/parameter_descriptor_builder.hpp"
#include "node_parameters/set_parameters_result_builder.hpp"
#include "node_parameters/validate_parameter.hpp"

namespace node_parameters {

class NodeParameters;

class ParameterSet {
 public:
  ParameterSet(std::string ns) : ns_(ns) {}
  virtual ~ParameterSet() = default;

  // Declare the parameters using ros interfaces ( called by NodeParameters )
  virtual bool declare(NodeParameters* node_parameters,
                       std::shared_ptr<rclcpp::Node> node) = 0;
  // Get the parameters using ros interfaces
  virtual bool get(std::shared_ptr<rclcpp::Node> node) = 0;

  const std::string getNamespace() const { return ns_; }

  // void setChangedCallback() const;
  // void registerSetChangedCallback(std::function<void()> callback) const;

 private:
  // namespace of parameter set
  std::string ns_;

  // mutable std::mutex mutex_;
  // mutable std::vector<std::function<void()>> changed_callback_;
};

class NodeParameters {
 public:
  NodeParameters(std::shared_ptr<rclcpp::Node> node) : node_(node) {}

  // T must be a class derived from ParameterSet
  template <typename T>
  bool declare(const std::string ns);
  template <typename T>
  const T get(const std::string ns);
  template <typename T>
  const T declare_and_get(const std::string ns);

  void registerValidateFunction(std::string name, ValidateFunction validate);

 private:
  const std::shared_ptr<rclcpp::Node> node_;
  // const rclcpp::Logger logger_;

  // sycronized access to internal state below
  std::mutex mutex_;
  std::map<std::string, std::unique_ptr<ParameterSet>> parameter_sets_;

  // map parameter name to validation callbacks
  std::map<std::string, std::vector<ValidateFunction>> validate_functions_;

  // map of parmaeter set modified change notify callbacks
  // std::map<std::string, std::function<void()>> set_changed_callbacks_;

  // ros2 handle for the OnSetParametersCallbackHandle
  // rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
  // on_set_callback_handle_;
};

template <typename T>
bool NodeParameters::declare(const std::string ns) {
  static_assert(std::is_base_of<ParameterSet, T>::value,
                "T must inherit from ParameterSet");

  std::lock_guard<std::mutex> lock(mutex_);

  // TODO handle parameter set already in map
  parameter_sets_[ns] = std::make_unique<T>(ns);

  // TODO handle parameter set is already declared
  return parameter_sets_[ns]->declare(this, node_);
}

template <typename T>
const T NodeParameters::get(const std::string ns) {
  static_assert(std::is_base_of<ParameterSet, T>::value,
                "T must inherit from ParameterSet");

  std::lock_guard<std::mutex> lock(mutex_);

  auto& parameter_set = parameter_sets_[ns];

  // TODO handle parameter set is not in map
  parameter_set->get(node_);

  // return a copy of the parameter set we just updated
  return *dynamic_cast<T*>(parameter_set.get());
}

template <typename T>
const T NodeParameters::declare_and_get(std::string ns) {
  static_assert(std::is_base_of<ParameterSet, T>::value,
                "T must inherit from ParameterSet");
  declare<T>(ns);
  return get<T>(ns);
}

}  // namespace node_parameters
