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

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <string>

namespace parameter_set {

class SetParametersResultBuilder {
  rcl_interfaces::msg::SetParametersResult msg_;

 public:
  /**
   * @brief      Constructs a SetParametersResultBuilder
   *
   * @param[in]  successful  The successful flag
   */
  SetParametersResultBuilder(bool successful) { msg_.successful = successful; }

  /**
   * @brief      Rcl_interfaces::msg::setparametersresult conversion operator.
   */
  operator rcl_interfaces::msg::SetParametersResult() const {
    return move(msg_);
  }

  /**
   * @brief      Set the reason string
   *
   * @param[in]  reason  The reason
   *
   * @return     Reference to this object
   */
  SetParametersResultBuilder& reason(std::string reason) {
    msg_.reason = reason;
    return *this;
  }
};

}  // namespace parameter_set
