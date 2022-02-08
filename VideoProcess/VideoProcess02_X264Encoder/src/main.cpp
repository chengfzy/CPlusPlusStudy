/**
 * @brief Video Encoder using libx264
 *
 * Ref:
 *  [1] https://blog.csdn.net/leixiaohua1020/article/details/42078645
 *  [2] https://www.videolan.org/developers/x264.html
 */
#include <fmt/format.h>
#include <glog/logging.h>
#include <cxxopts.hpp>
#include "VideoEncoder.h"

using namespace std;
using namespace fmt;
namespace fs = boost::filesystem;

void splitFrame() {
    fs::path videoFile("/home/jeffery/Documents/Code/Others/CPlusPlusStudy/data/cuc_ieschool_640x360_yuv420p.yuv");
    fs::path imageFolder("/home/jeffery/Data/YUV420");
    constexpr int width{640}, height{360};
    int ySize = width * height;  // YUV420
    int uSize = ySize / 4;
    int vSize = ySize / 4;
    int chunkSize = ySize + uSize + vSize;
    LOG(INFO) << format("Y/U/V size = {}/{}/{}, chunk size = {}", ySize, uSize, vSize, chunkSize);

    // open video file
    ifstream inFs(videoFile.string(), ios::binary);
    CHECK(inFs.is_open()) << format("cannot open YUV file \"{}\"", videoFile.string());

    // create save folder
    fs::create_directories(imageFolder);

    // calculate frame number
    inFs.seekg(0, ios::end);
    const int kFrameCount = inFs.tellg() / chunkSize;
    LOG(INFO) << format("frame count = {}", kFrameCount);

    // read file and save to file
    inFs.seekg(0, ios::beg);
    vector<char> raw(chunkSize);
    for (int i = 0; i < kFrameCount; ++i) {
        LOG(INFO) << format("process frame [{}/{}]", i, kFrameCount);

        // read data
        inFs.read(raw.data(), chunkSize);

        // save to file
        fs::path imageFile = imageFolder / format("{}.bin", i);
        ofstream outFs(imageFile.string(), ios::binary);
        CHECK(outFs.is_open()) << format("cannot open \"{}\" to save H264 video", imageFile.string());
        outFs.write(raw.data(), raw.size());
        outFs.close();
    }
    inFs.close();
}

int main(int argc, const char* argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;

    // splitFrame();
    // return 0;

    // argument parser
    cxxopts::Options options(argv[0], "Video Encoder & Decoder");
    options.set_width(120);
    // clang-format off
    options.add_options()
        ("imageFolder", "image folder", cxxopts::value<string>()->default_value("./data/images"))
        ("width", "image width", cxxopts::value<int>()->default_value("1920"))
        ("height", "image height", cxxopts::value<int>()->default_value("1080"))
        ("saveFile", "save video file", cxxopts::value<string>()->default_value("./data/video.264"))
        ("h,help", "help message");
    // clang-format on
    auto result = options.parse(argc, argv);
    if (result.count("help")) {
        cout << options.help() << endl;
        return false;
    }

    // parse
    fs::path imageFolder = fs::weakly_canonical(fs::path(result["imageFolder"].as<string>()));
    int width = result["width"].as<int>();
    int height = result["height"].as<int>();
    fs::path saveFile = fs::weakly_canonical(fs::path(result["saveFile"].as<string>()));
    LOG(INFO) << format("image folder: {}", imageFolder.string());
    LOG(INFO) << format("image size: {}x{}", width, height);
    LOG(INFO) << format("save video file: {}", saveFile.string());

    VideoEncoder encoder;
    // encoder.init(imageFolder, width, height, X264_CSP_I422);
    // encoder.init(imageFolder, width, height, X264_CSP_I420);
    encoder.init(imageFolder, width, height, X264_CSP_YUYV);
    // encoder.saveYUVPlanar("/home/jeffery/Data/YUV/testPlanar");
    // encoder.show(10);
    // encoder.saveJpeg();
    encoder.encode(saveFile);

    google::ShutdownGoogleLogging();
    return 0;
}