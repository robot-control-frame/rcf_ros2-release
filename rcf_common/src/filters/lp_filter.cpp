/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * SPDX-License-Identifier: Apache-2.0
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by qiayuan on 1/5/21.
//
#include "rcf_common/filters/lp_filter.hpp"

#include <cmath>
#include <rclcpp/logging.hpp>
namespace rcf_common {
namespace filters {
LowPassFilter::LowPassFilter(double cutoff_freq) : delta_t_(0.0) { cutoff_frequency_ = cutoff_freq; }

void LowPassFilter::input(double in, double dt) {
  // My filter reference was Julius O. Smith III, Intro. to Digital Filters
  // With Audio Applications.
  // See https://ccrma.stanford.edu/~jos/filters/Example_Second_Order_Butterworth_Lowpass.html
  in_[2] = in_[1];
  in_[1] = in_[0];
  in_[0] = in;

  if (dt <= 0.0) {
    RCLCPP_ERROR(rclcpp::get_logger("LowPassFilter"), "dt is <= 0 (%f), skipping this loop.", dt);
    return;
  }
  delta_t_ = dt;

  if (cutoff_frequency_ != -1 && cutoff_frequency_ > 0) {
    // Check if tan(_) is really small, could cause c = NaN
    tan_filt_ = tan((cutoff_frequency_ * 6.2832) * delta_t_ / 2.);
    // Avoid tan(0) ==> NaN
    if ((tan_filt_ <= 0.) && (tan_filt_ > -0.01)) tan_filt_ = -0.01;
    if ((tan_filt_ >= 0.) && (tan_filt_ < 0.01)) tan_filt_ = 0.01;
    c_ = 1 / tan_filt_;
  }

  out_[2] = out_[1];
  out_[1] = out_[0];
  out_[0] = (1 / (1 + c_ * c_ + M_SQRT2 * c_)) *
            (in_[2] + 2 * in_[1] + in_[0] - (c_ * c_ - M_SQRT2 * c_ + 1) * out_[2] - (-2 * c_ * c_ + 2) * out_[1]);
}

double LowPassFilter::output() { return out_[0]; }

void LowPassFilter::reset() {
  for (int i = 0; i < 3; ++i) {
    in_[i] = 0;
    out_[i] = 0;
  }
}
}  // namespace filters
}  // namespace rcf_common
