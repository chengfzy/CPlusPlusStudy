#pragma once
#include <fmt/ostream.h>
#include <boost/filesystem.hpp>

template <>
struct fmt::formatter<boost::filesystem::path> : fmt::ostream_formatter {};
