#include <fmt/format.h>
#include <fmt/ostream.h>
#include <fmt/ranges.h>
#include <glog/logging.h>
#include <boost/algorithm/string.hpp>
#include <cxxopts.hpp>
#include <iostream>
#include "VideoEncoder.h"

using namespace std;
using namespace fmt;
namespace fs = boost::filesystem;

int main(int argc, const char* argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;

    // argument parser
    cxxopts::Options options(argv[0], "Video Encoder using FFmpeg");
    options.set_width(120);
    // clang-format off
    options.add_options()
        ("imageFolder", "image folder", cxxopts::value<string>()->default_value("./data/images"))
        ("imageFormat", "image format", cxxopts::value<string>()->default_value("YUYV422"))
        ("width", "image width", cxxopts::value<int>()->default_value("1920"))
        ("height", "image height", cxxopts::value<int>()->default_value("1080"))
        ("saveFile", "save video file", cxxopts::value<string>()->default_value("./data/video.h264"))
        ("h,help", "help message");
    // clang-format on
    auto result = options.parse(argc, argv);
    if (result.count("help")) {
        cout << options.help() << endl;
        return false;
    }

    // parse
    fs::path imageFolder = fs::weakly_canonical(fs::path(result["imageFolder"].as<string>()));
    // parse and check image format
    string imageFormatStr = result["imageFormat"].as<string>();
    vector<string> imageFormats = {"YUV420P", "YUV422P", "YUYV422"};
    auto it = find_if(imageFormats.begin(), imageFormats.end(),
                      [&](const string& v) { return boost::iequals(v, imageFormatStr); });
    if (it == imageFormats.end()) {
        cout << format("input image format should be one item in {}", imageFormats) << endl;
        cout << options.help() << endl;
        return 0;
    }
    ImageFormat imageFormat{ImageFormat::None};
    if (imageFormatStr == "YUV420P") {
        imageFormat = ImageFormat::YUV420P;
    } else if (imageFormatStr == "YUV422P") {
        imageFormat = ImageFormat::YUV422P;
    } else if (imageFormatStr == "YUYV422") {
        imageFormat = ImageFormat::YUYV422;
    }
    int width = result["width"].as<int>();
    int height = result["height"].as<int>();
    fs::path saveFile = fs::weakly_canonical(fs::path(result["saveFile"].as<string>()));
    LOG(INFO) << format("image folder: {}", imageFolder.string());
    LOG(INFO) << format("image format: {}", imageFormat);
    LOG(INFO) << format("image size: {}x{}", width, height);
    LOG(INFO) << format("save video file: {}", saveFile.string());

    VideoEncoder encoder;
    encoder.init(imageFolder, width, height, imageFormat);
    // encoder.show();
    encoder.encode(saveFile);

    google::ShutdownGoogleLogging();
    return 0;
}