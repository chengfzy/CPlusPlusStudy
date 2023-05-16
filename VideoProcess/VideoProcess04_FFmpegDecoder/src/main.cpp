#include <fmt/format.h>
#include <fmt/ranges.h>
#include <glog/logging.h>
#include <boost/algorithm/string.hpp>
#include <cxxopts.hpp>
#include <iostream>
#include "VideoDecoder.h"

using namespace std;
using namespace fmt;
namespace fs = boost::filesystem;

int main(int argc, const char* argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;

    // argument parser
    cxxopts::Options options(argv[0], "Video Decoder using FFmpeg");
    options.set_width(120);
    // clang-format off
    options.add_options()
        ("videoFile", "video file", cxxopts::value<string>()->default_value("./data/video.264"))
        ("saveFolder", "image saving folder", cxxopts::value<string>()->default_value("./data/images"))
        ("h,help", "help message");
    // clang-format on
    auto result = options.parse(argc, argv);
    if (result.count("help")) {
        cout << options.help() << endl;
        return false;
    }

    // parse
    fs::path videoFile = fs::weakly_canonical(fs::path(result["videoFile"].as<string>()));
    fs::path saveFolder = fs::weakly_canonical(fs::path(result["saveFolder"].as<string>()));
    LOG(INFO) << format("video file: {}", videoFile.string());
    LOG(INFO) << format("image saving folder: {}", saveFolder.string());

    VideoDecoder decoder;
    // decoder.init(saveFolder, width, height);
    decoder.decode(videoFile, saveFolder);

    google::ShutdownGoogleLogging();
    return 0;
}