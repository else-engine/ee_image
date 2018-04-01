/**
 * Copyright (c) 2017-2018 Gauthier ARNOULD
 * This file is released under the zlib License (Zlib).
 * See file LICENSE or go to https://opensource.org/licenses/Zlib
 * for full license details.
 */

#pragma once

#include <cmath>

#include <ee_utils/templates.hpp>

namespace ee {

using tutil::eif;

// http://entropymine.com/imageworsener/srgbformula/
template <typename T>
constexpr eif<std::is_floating_point_v<T>, T> srgb_to_linear(T value) {
#if 0
    if (value <= T{0.04045L}) {
#else
    if (value <= T{0.0404482362771082L}) {
#endif
        return value / T{12.92L};
    }

    return std::pow((value + T{0.055L}) / T{1.055L}, T{2.4L});
}

template <typename T>
constexpr eif<std::is_floating_point_v<T>, T> linear_to_srgb(T value) {
#if 0
    if (value <= T{0.0031308L}) {
#else
    if (value <= T{0.00313066844250063L}) {
#endif
        return value * T{12.92L};
    }

    return std::pow(value, T{1.0L} / T{2.4L}) * T{1.055L} - T{0.055L};
}

template <typename T>
constexpr eif<std::is_floating_point_v<T>, T> gamma_to_linear(T value, T gamma) {
    return std::pow(value, gamma);
}

template <typename T>
constexpr eif<std::is_floating_point_v<T>, T> linear_to_gamma(T value, T gamma) {
    return std::pow(value, T{1.0L} / gamma);
}

} // namespace ee
