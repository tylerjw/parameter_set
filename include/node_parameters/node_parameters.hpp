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

using validate::ValidateFunction;
class NodeParameters;

struct ParameterSet {
 public:
  /**
   * @brief      Interface for sets of parameters
   * @details    See example for how to create your own ParameterSets
   *
   * @param[in]  ns    The namespace
   */
  ParameterSet(std::string ns) : ns_(ns) {}

  /**
   * @brief      Destroys the object.
   */
  virtual ~ParameterSet() = default;

  /**
   * @brief      Interface for declaring parameters called by NodeParameters
   *
   * @param[in]   node_parameters   Pointer to NodeParameters object for calling
   * registerValidateFunction
   * @param[in]   node              Node for calling ros interfaces for
   * declaring parameters
   * @return     true on success
   */
  virtual bool declare(NodeParameters* node_parameters,
                       std::shared_ptr<rclcpp::Node> node) = 0;

  /**
   * @brief      Interface for getting parameters in the set called by
   * NodeParameters
   *
   * @param[in]  node  The node
   *
   * @return     true on success
   */
  virtual bool get(std::shared_ptr<rclcpp::Node> node) = 0;

  /**
   * @brief      Gets the namespace.
   *
   * @return     The namespace.
   */
  const std::string getNamespace() const { return ns_; }

 private:
  // namespace of parameter set
  std::string ns_;
};

class NodeParameters {
 public:
  /**
   * @brief      Class for interfacing with ROS2 parameters
   *
   * @param[in]  node  The node
   */
  NodeParameters(std::shared_ptr<rclcpp::Node> node);

  /**
   * @brief      Destroys the object.
   */
  ~NodeParameters();

  /**
   * @brief      Declare ParameterSet
   *
   * @param[in]  ns    The namespace to declare the parameters into
   *
   * @tparam     T     must be derived from ParameterSet
   *
   * @return     true on success
   */
  template <typename T>
  bool declare(const std::string ns);

  /**
   * @brief      Get ParameterSet
   *
   * @param[in]  ns    The namespace to get from
   *
   * @tparam     T     must be derived from ParameterSet and must be declared in
   * the namespace
   *
   * @return     object derived from ParameterSet
   */
  template <typename T>
  const T get(const std::string ns);

  /**
   * @brief      Declare and Get ParameterSet
   *
   * @param[in]  ns    The namespace to declare and get from
   *
   * @tparam     T     must be derived from ParameterSet
   *
   * @return     object derived from ParameterSet
   */
  template <typename T>
  const T declare_and_get(const std::string ns);

  /**
   * @brief      Register validation function for a given parameter
   * @details    Can be called many times to add many validation functions to
   * one parameter
   *
   * @param[in]  name      The name of the function
   * @param[in]  validate  The validation function
   */
  void registerValidateFunction(std::string name, ValidateFunction validate);

  /**
   * @brief      Register callback for a set namespace for when parameters in
   * that set are changed
   * @details    Stored in a 1:1 relationship between name and callback.
   *
   * @param[in]  name      The namespace
   * @param[in]  callback  The callback
   */
  void registerSetChangedCallback(std::string name,
                                  std::function<void()> callback);

 private:
  const std::shared_ptr<rclcpp::Node> node_;

  // sycronized access to internal state below
  mutable std::recursive_mutex mutex_;
  std::map<std::string, std::unique_ptr<ParameterSet>> parameter_sets_;

  // map parameter name to validation callbacks
  std::map<std::string, std::vector<ValidateFunction>> validate_functions_;

  // map of parmaeter set modified change notify callbacks
  std::thread set_changed_callback_thread_;
  std::map<std::string, std::function<void()>> set_changed_callbacks_;

  // ros2 handle for the OnSetParametersCallbackHandle
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      on_set_callback_handle_;

  /**
   * @brief ros2 parameter validation/update callback
   * @param parameters - vector of parameters
   * @return result is set to false if parameters are not validated
   * @details All the parameters all validated, then the set changed callbacks
   * are called to notify them
   */
  rcl_interfaces::msg::SetParametersResult setParametersCallback(
      const std::vector<rclcpp::Parameter>& parameters);

  /**
   * @brief helper function for validating a parameter
   * @param parameter ros2 parameter to validate
   * @return result is set to false if parameter validation fails
   */
  rcl_interfaces::msg::SetParametersResult validateParameter(
      const rclcpp::Parameter& parameter) const;
};

template <typename T>
bool NodeParameters::declare(const std::string ns) {
  static_assert(std::is_base_of<ParameterSet, T>::value,
                "T must inherit from ParameterSet");

  std::lock_guard<std::recursive_mutex> lock(mutex_);

  // TODO handle parameter set already in map
  parameter_sets_[ns] = std::make_unique<T>(ns);

  // TODO handle parameter set is already declared
  return parameter_sets_[ns]->declare(this, node_);
}

template <typename T>
const T NodeParameters::get(const std::string ns) {
  static_assert(std::is_base_of<ParameterSet, T>::value,
                "T must inherit from ParameterSet");

  std::lock_guard<std::recursive_mutex> lock(mutex_);

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

/**
 * @breif Splits a string on the first dot `.`.  Used to get parmaeter
 * namespace.
 *
 * @param[in]   full_name   The full parameter name including namespace
 * @return     {namespace, parameter_name} or {"", full_name} if `.` is not
 * found in full_name
 */
std::pair<std::string, std::string> split_parameter_name(
    const std::string& full_name);

}  // namespace node_parameters
