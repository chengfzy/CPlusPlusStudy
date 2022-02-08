/**
 * @brief Simple Video Encoder using libx264
 *
 * Ref:
 *  [1] https://blog.csdn.net/leixiaohua1020/article/details/42078645
 *  [2] https://www.videolan.org/developers/x264.html
 */

#include <fmt/format.h>
#include <glog/logging.h>
#include <x264.h>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace fmt;
using namespace std::chrono;
namespace fs = boost::filesystem;

int main(int argc, char* argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;

    // input
    fs::path imageFolder("/home/jeffery/Data/YUV");      // image folder
    constexpr int width{1920}, height{1080};             // image size
    fs::path saveFile("/home/jeffery/Data/video.h264");  // save video file
    LOG(INFO) << format("image folder: {}", imageFolder.string());
    LOG(INFO) << format("image size = {}x{}", width, height);
    LOG(INFO) << format("save video file: {}", saveFile.string());

    // list images
    vector<fs::path> imageFiles;
    fs::directory_iterator itEnd;
    for (fs::directory_iterator it(imageFolder); it != itEnd; ++it) {
        if (!is_directory(*it)) {
            imageFiles.emplace_back(fs::canonical(it->path()));
        }
    }
    // sort
    sort(imageFiles.begin(), imageFiles.end(), [](const fs::path& p1, const fs::path& p2) {
        using namespace boost;
        auto id1 = lexical_cast<uint64_t>(replace_all_copy(trim_copy(p1.stem().string()), "yuv", ""));
        auto id2 = lexical_cast<uint64_t>(replace_all_copy(trim_copy(p2.stem().string()), "yuv", ""));
        return id1 < id2;
    });
    // print
    for (size_t i = 0; i < imageFiles.size(); ++i) {
        cout << format("\t[{}/{}] {}", i, imageFiles.size(), imageFiles[i].string()) << endl;
    }

    // YUV size, YUV422
    int ySize = width * height;
    int uSize = ySize / 2;
    int vSize = ySize / 2;
    int chunkSize = ySize + uSize + vSize;
    LOG(INFO) << format("Y/U/V size = {}/{}/{}, chunk size = {}", ySize, uSize, vSize, chunkSize);

#if 0
    // read image and show
    for (size_t i = 0; i < imageFiles.size(); ++i) {
        string fileName = imageFiles[i].string();
        fstream fs(fileName, ios::in | ios::binary);
        if (!fs.is_open()) {
            LOG(FATAL) << format("cannot open image file \"{}\"", fileName);
            return 0;
        }
        vector<unsigned char> raw = vector<unsigned char>(istreambuf_iterator<char>(fs), {});
        fs.close();

        // to BGR and show
        cv::Mat yuv(height, width, CV_8UC2, raw.data());
        cv::Mat bgr;
        cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_YUYV);
        cv::imshow("BGR", bgr);
        cv::waitKey(100);
    }
#endif

    // compress
    int csp = X264_CSP_I422;  // YUV422
    // set parameters of x246
    x264_param_t param;
    x264_param_default(&param);
    param.i_log_level = X264_LOG_DEBUG;
    param.i_width = width;
    param.i_height = height;
    param.i_threads = X264_SYNC_LOOKAHEAD_AUTO;
    param.i_frame_total = 0;
    param.i_keyint_max = 10;
    param.i_bframe = 0;
    param.b_open_gop = 0;
    param.i_bframe_pyramid = 0;
    param.rc.i_qp_constant = 0;
    param.rc.i_qp_max = 0;
    param.rc.i_qp_min = 0;
    param.i_bframe_adaptive = X264_B_ADAPT_TRELLIS;
    param.i_fps_den = 1;
    param.i_fps_num = 25;
    param.i_timebase_den = param.i_fps_num;
    param.i_timebase_num = param.i_fps_den;
    param.i_csp = csp;
    // x264_param_apply_profile(&param, x264_profile_names[5]);

    // open yuv and h264 file
    ofstream outFs(saveFile.string(), ios::binary);
    CHECK(outFs.is_open()) << format("cannot open \"{}\" to save H264 video", saveFile.string());

    // alloc memory for input image
    x264_picture_t inPic;
    x264_picture_alloc(&inPic, csp, width, height);
    // out image
    x264_picture_t outPic;
    x264_picture_init(&outPic);
    x264_nal_t* nal = nullptr;
    int iNal{0};

    // init encoder
    x264_t* handle = x264_encoder_open(&param);
    CHECK(handle != nullptr) << "cannot init encoder";

    // encode to H264
    for (int i = 0; i < min(50UL, imageFiles.size()); ++i) {
        LOG(INFO) << format("encoding frame [{}]", i);

        // read image
        string fileName = imageFiles[i].string();
        fstream fs(fileName, ios::in | ios::binary);
        if (!fs.is_open()) {
            LOG(FATAL) << format("cannot open image file \"{}\"", fileName);
            return 0;
        }
        vector<unsigned char> raw = vector<unsigned char>(istreambuf_iterator<char>(fs), {});
        fs.close();

        // convert YUYV(YUV422 packed) to YUV(YUV422 planar)
        inPic.i_pts = i;
        unsigned char* pRaw = raw.data();
        uint8_t* pY = inPic.img.plane[0];
        uint8_t* pU = inPic.img.plane[1];
        uint8_t* pV = inPic.img.plane[2];
        for (int m = 0; m < uSize; ++m) {
            *(pY++) = *(pRaw++);
            *(pU++) = *(pRaw++);
            *(pY++) = *(pRaw++);
            *(pV++) = *(pRaw++);
        }

        // encode
        int ret = x264_encoder_encode(handle, &nal, &iNal, &inPic, &outPic);
        LOG_IF(ERROR, ret < 0) << format("encode error: {}", ret);

        // write to file
        for (int j = 0; j < iNal; ++j) {
            outFs.write(reinterpret_cast<char*>(nal[j].p_payload), nal[j].i_payload);
        }
    }
    // flush encoder
    while (true) {
        int ret = x264_encoder_encode(handle, &nal, &iNal, nullptr, &outPic);
        if (ret == 0) {
            break;
        }
        LOG(INFO) << format("flush 1 frame");
        // write to file
        for (int j = 0; j < iNal; ++j) {
            outFs.write(reinterpret_cast<char*>(nal[j].p_payload), nal[j].i_payload);
        }
    }

    // clear picture and close encoder
    x264_picture_clean(&inPic);
    x264_encoder_close(handle);

    // close file
    outFs.close();

    google::ShutdownGoogleLogging();
    return 0;
}