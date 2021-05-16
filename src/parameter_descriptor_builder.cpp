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

#include "node_parameters/parameter_descriptor_builder.hpp"

namespace node_parameters {

ParameterDescriptorBuilder& ParameterDescriptorBuilder::type(uint8_t type) {
  msg_.type = type;
  return *this;
}

ParameterDescriptorBuilder& ParameterDescriptorBuilder::description(
    std::string description) {
  msg_.description = description;
  return *this;
}

ParameterDescriptorBuilder& ParameterDescriptorBuilder::additional_constraints(
    std::string additional_constraints) {
  msg_.additional_constraints = additional_constraints;
  return *this;
}

ParameterDescriptorBuilder& ParameterDescriptorBuilder::read_only(
    bool read_only) {
  msg_.read_only = read_only;
  return *this;
}

// In rolling (not in Foxy)
// ParameterDescriptorBuilder& ParameterDescriptorBuilder::dynamic_typing(bool
// dynamic_typing)
// {
//   msg_.dynamic_typing = dynamic_typing;
//   return *this;
// }

ParameterDescriptorBuilder& ParameterDescriptorBuilder::floating_point_range(
    double from_value /*= std::numeric_limits<double>::min()*/,
    double to_value /*= std::numeric_limits<double>::max()*/,
    double step /*= 0*/) {
  msg_.floating_point_range.resize(1);
  msg_.floating_point_range[0].from_value = from_value;
  msg_.floating_point_range[0].to_value = to_value;
  msg_.floating_point_range[0].step = step;
  return *this;
}

ParameterDescriptorBuilder& ParameterDescriptorBuilder::integer_range(
    int64_t from_value /*= std::numeric_limits<int64_t>::min()*/,
    int64_t to_value /*= std::numeric_limits<int64_t>::max()*/,
    int64_t step /*= 0*/) {
  msg_.integer_range.resize(1);
  msg_.integer_range[0].from_value = from_value;
  msg_.integer_range[0].to_value = to_value;
  msg_.integer_range[0].step = step;
  return *this;
}

}  // namespace node_parameters
