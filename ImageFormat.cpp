/**
 * Copyright (c) 2017-2018 Gauthier ARNOULD
 * This file is released under the zlib License (Zlib).
 * See file LICENSE or go to https://opensource.org/licenses/Zlib
 * for full license details.
 */

#include "ImageFormat.hpp"

#include <cstring>

#include <ee_utils/templates.hpp>
#include <ee_utils/scoped_enum_utils.hpp>
#include <ee_math/ee_math.hpp>

#include "color_utils.hpp"

namespace ee {

using tutil::eif;

/**
 * Map signed interger range to signed normalized floating point [-1.0, 1.0].
 */
template <typename Tout, typename Tin>
constexpr eif<std::is_integral_v<Tin> && std::is_signed_v<Tin> && std::is_floating_point_v<Tout>, Tout> SI_to_SN(Tin s) {
    // As OpenGL does :
    // https://www.opengl.org/wiki/Normalized_Integer
    return math::max(static_cast<Tout>(s) / std::numeric_limits<Tin>::max(), - Tout{1L});
}

/**
 * Map signed interger range to unsigned normalized floating point [0.0, 1.0].
 */
template <typename Tout, typename Tin>
constexpr eif<std::is_integral_v<Tin> && std::is_signed_v<Tin> && std::is_floating_point_v<Tout>, Tout> SI_to_UN(Tin s) {
    // As OpenGL does :
    // https://www.opengl.org/wiki/Normalized_Integer
    return Tout{0.5L} * math::max(static_cast<Tout>(s) / std::numeric_limits<Tin>::max(), - Tout{1L}) + Tout{0.5L};
}

/**
 * Map unsigned interger range to unsigned normalized floating point [0.0, 1.0].
 */
template <typename Tout, typename Tin>
constexpr eif<std::is_integral_v<Tin> && ! std::is_signed_v<Tin> && std::is_floating_point_v<Tout>, Tout> UI_to_UN(Tin s) {
    return static_cast<Tout>(s) / std::numeric_limits<Tin>::max();
}

/**
 * Map unsigned interger range to signed normalized floating point [-1.0, 1.0].
 */
template <typename Tout, typename Tin>
constexpr eif<std::is_integral_v<Tin> && ! std::is_signed_v<Tin> && std::is_floating_point_v<Tout>, Tout> UI_to_SN(Tin s) {
    return Tout{2L} * (static_cast<Tout>(s) / std::numeric_limits<Tin>::max() - Tout{0.5L});
}

/**
 * Map unsigned normalized floating point to unsigned integer range.
 */
template <typename Tout, typename Tin>
constexpr eif<std::is_floating_point_v<Tin> && std::is_integral_v<Tout> && ! std::is_signed_v<Tout>, Tout> UN_to_UI(Tin s) {
    return math::round(Tin{std::numeric_limits<Tout>::max()} * s);
}

/**
 * Map signed normalized floating point to unsigned integer range.
 */
template <typename Tout, typename Tin>
constexpr eif<std::is_floating_point_v<Tin> && std::is_integral_v<Tout> && ! std::is_signed_v<Tout>, Tout> SN_to_UI(Tin s) {
    return math::round(Tin{std::numeric_limits<Tout>::max()} * (s * Tin{0.5L} + Tin{0.5L}));
}

template <std::size_t S>
using uint = eif<S == 8 || S == 16 || S == 32 || S == 64,
      std::conditional_t<S == 8, std::uint8_t,
      std::conditional_t<S == 16, std::uint16_t,
      std::conditional_t<S == 32, std::uint32_t,
      std::uint64_t>>>>;

template <std::size_t S>
class UI {
    public:
        static std::uint64_t get_ui(byte* data, std::size_t c) {
            return get(data, c);
        }

        static void set_ui(byte* data, std::size_t c, std::uint64_t value) {
            set(data, c, value);
        }

        static double get_un(byte* data, std::size_t c) {
            return UI_to_UN<double>(get(data, c));
        }

        static void set_un(byte* data, std::size_t c, double value) {
            set(data, c, UN_to_UI<uint<S>>(value));
        }

        static double get_sn(byte* data, std::size_t c) {
            return 2.0 * (get_un(data, c) - 0.5);
        }

        static void set_sn(byte* data, std::size_t c, double value) {
            set_un(data, c, 0.5 * value + 0.5);
        }

    private:
        static constexpr std::size_t size{S / 8};

        static uint<S> get(byte* data, std::size_t c) {
            uint<S> val;

            std::memcpy(&val, data + c * size, size);

            return val;
        }

        static void set(byte* data, std::size_t c, uint<S> value) {
            std::memcpy(data + c * size, &value, size);
        }
};

class UI_SRGB : public UI<8> {
    public:
        static double get_un(byte* data, std::size_t c) {
            double value = UI<8>::get_un(data, c);

            return c < 3 ? srgb_to_linear(value) : value;
        }

        static void set_un(byte* data, std::size_t c, double value) {
            UI<8>::set_un(data, c, c < 3 ? linear_to_srgb(value) : value);
        }

        static double get_sn(byte* data, std::size_t c) {
            return 2.0 * (get_un(data, c) - 0.5);
        }

        static void set_sn(byte* data, std::size_t c, double value) {
            set_un(data, c, 0.5 * value + 0.5);
        }
};

template <typename F>
const PixelAccessors pixel_accessors_from() {
    return PixelAccessors{
        F::get_ui,
        F::set_ui,

        F::get_un,
        F::set_un,

        F::get_sn,
        F::set_sn
    };
}

struct ImageFormatInfo {
    const std::uint8_t components;
    const std::uint8_t size;
    const PixelAccessors accessors;
};

const ImageFormatInfo if_info[] = {
    {1, 1, pixel_accessors_from<UI<8>>()},   // R8_UI
    {2, 2, pixel_accessors_from<UI<8>>()},   // R8G8_UI
    {3, 3, pixel_accessors_from<UI<8>>()},   // R8G8B8_UI
    {4, 4, pixel_accessors_from<UI<8>>()},   // R8G8B8A8_UI
    {3, 3, pixel_accessors_from<UI_SRGB>()}, // R8G8B8_UI_SRGB
    {4, 4, pixel_accessors_from<UI_SRGB>()}, // R8G8B8A8_UI_SRGB
};

static const ImageFormatInfo& get(ImageFormat imgf) {
    return if_info[as_value(imgf)];
}

std::uint8_t components(ImageFormat imgf) {
    return get(imgf).components;
}

std::uint8_t size(ImageFormat imgf) {
    return get(imgf).size;
}

bool is_srgb(ImageFormat imgf) {
    return imgf == ImageFormat::R8G8B8_UI_SRGB || imgf == ImageFormat::R8G8B8A8_UI_SRGB;
}

const PixelAccessors& accessors(ImageFormat imgf) {
    return get(imgf).accessors;
}

} // namespace ee
