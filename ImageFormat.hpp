/**
 * Copyright (c) 2017-2018 Gauthier ARNOULD
 * This file is released under the zlib License (Zlib).
 * See file LICENSE or go to https://opensource.org/licenses/Zlib
 * for full license details.
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include <ee_utils/templates.hpp>

namespace ee {

using std::byte;

struct PixelAccessors {
    std::uint64_t (*get_ui)(byte*, std::size_t);
    void (*set_ui)(byte*, std::size_t, std::uint64_t);

    double (*get_un)(byte*, std::size_t);
    void (*set_un)(byte*, std::size_t, double);

    double (*get_sn)(byte*, std::size_t);
    void (*set_sn)(byte*, std::size_t, double);
};

enum class ImageFormat {
    R8_UI,
    R8G8_UI,
    R8G8B8_UI,
    R8G8B8A8_UI,
    R8G8B8_UI_SRGB,
    R8G8B8A8_UI_SRGB,
};

std::uint8_t components(ImageFormat);
std::uint8_t size(ImageFormat);
bool is_srgb(ImageFormat);
const PixelAccessors& accessors(ImageFormat);

} // namespace ee
