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

#include "rcf_common/filters/ramp_filter.hpp"

#include "rcf_common/math_utilities.hpp"
namespace rcf_common {
namespace filters {
template <typename T>
RampFilter<T>::RampFilter(T acc, T dt) {
  acc_ = acc;
  dt_ = dt;
  RampFilter::clear();
}

template <typename T>
void RampFilter<T>::input(T input_value) {
  last_value_ += min_abs(input_value - last_value_, acc_ * dt_);
}

template <typename T>
void RampFilter<T>::clear() {
  last_value_ = 0.;
}

template <typename T>
void RampFilter<T>::clear(T last_value) {
  last_value_ = last_value;
}

template <typename T>
void RampFilter<T>::setAcc(T acc) {
  acc_ = acc;
}

template <typename T>
T RampFilter<T>::output() {
  return last_value_;
}

template class RampFilter<float>;
template class RampFilter<double>;

}  // namespace filters
}  // namespace rcf_common
