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

#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <string>

namespace node_parameters {

class ParameterDescriptorBuilder {
  rcl_interfaces::msg::ParameterDescriptor msg_;

 public:
  operator rcl_interfaces::msg::ParameterDescriptor() const {
    return move(msg_);
  }

  ParameterDescriptorBuilder& type(uint8_t type);
  ParameterDescriptorBuilder& description(std::string description);
  ParameterDescriptorBuilder& additional_constraints(
      std::string additional_constraints);
  ParameterDescriptorBuilder& read_only(bool read_only);
  // Rolling only
  // ParameterDescriptorBuilder& dynamic_typing(bool dynamic_typing);
  ParameterDescriptorBuilder& floating_point_range(
      double from_value = std::numeric_limits<double>::min(),
      double to_value = std::numeric_limits<double>::max(), double step = 0);
  ParameterDescriptorBuilder& integer_range(
      int64_t from_value = std::numeric_limits<int64_t>::min(),
      int64_t to_value = std::numeric_limits<int64_t>::max(), int64_t step = 0);
};

}  // namespace node_parameters
