#pragma once
#include <fmt/format.h>

#ifndef FMT_FORMAT_ENUM

/**
 * @brief Macro to format enum class.
 *
 * override "<<" to print enum class when #include <fmt/ostream.h>
 *
 */
#define FMT_FORMAT_ENUM(EnumType, ...)                                                                                \
    inline std::ostream& operator<<(std::ostream& os, const EnumType& v) {                                            \
        static_assert(std::is_enum<EnumType>::value, #EnumType " must be an enum");                                   \
        static const std::pair<EnumType, std::string_view> m[] = __VA_ARGS__;                                         \
        auto it = std::find_if(std::begin(m), std::end(m),                                                            \
                               [v](const std::pair<EnumType, std::string_view>& p) -> bool { return p.first == v; }); \
        os << (it != std::end(m) ? it->second : "");                                                                  \
        return os;                                                                                                    \
    }

#endif

/**
 * @brief Image format(colorspace)
 *
 */
enum class ImageFormat {
    None,     // unknown
    YUV420P,  // YUV 4:2:0 planar
    YUV422P,  // YUV 4:2:2 planar
    YUYV422,  // YUYV 4:2:2 packed
};

FMT_FORMAT_ENUM(ImageFormat, {{ImageFormat::None, "None"},
                              {ImageFormat::YUV420P, "YUV420P"},
                              {ImageFormat::YUV422P, "YUV422P"},
                              {ImageFormat::YUYV422, "YUYV422"}})
