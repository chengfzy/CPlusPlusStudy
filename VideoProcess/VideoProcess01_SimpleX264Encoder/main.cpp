/**
 * @brief Simple Video Encoder using libx264
 *
 * Ref:
 *  [1] https://blog.csdn.net/leixiaohua1020/article/details/42078645
 *  [2] https://www.videolan.org/developers/x264.html
 */

#include <fmt/format.h>
#include <glog/logging.h>
#include <sched.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

#ifdef __cplusplus
extern "C" {
#endif
#include <x264.h>
#ifdef __cplusplus
};
#endif

using namespace std;
using namespace fmt;
using namespace std::chrono;

int main(int argc, char* argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;

    // string inFileName{"./data/cuc_ieschool_640x360_yuv444p.yuv"};
    // int csp = X264_CSP_I444;  // YUV444
    string inFileName{"./data/cuc_ieschool_640x360_yuv420p.yuv"};
    int csp = X264_CSP_I420;  // YUV420
    string outFileName{"./data/out.h264"};

    // open yuv and h264 file
    ifstream inFs(inFileName, ios::binary);
    CHECK(inFs.is_open()) << format("cannot open YUV file \"{}\"", inFileName);
    ofstream outFs(outFileName, ios::binary);
    CHECK(outFs.is_open()) << format("cannot open \"{}\" to save H264 video", outFileName);

    // image size
    constexpr int width{640}, height{360};
    int ySize{0}, uSize{0}, vSize{0};
    switch (csp) {
        case X264_CSP_I444:  // YUV444P
            ySize = width * height;
            uSize = ySize;
            vSize = ySize;
            break;
        case X264_CSP_I420:  // YUV420P
            ySize = width * height;
            uSize = ySize / 4;
            vSize = ySize / 4;
            break;
        default:
            LOG(ERROR) << format("unsupported format {}", csp);
            break;
    }
    int chunkSize = ySize + uSize + vSize;
    LOG(INFO) << format("Y/U/V size = {}/{}/{}, chunk size = {}", ySize, uSize, vSize, chunkSize);

    // calculate frame number
    inFs.seekg(0, ios::end);
    const int kFrameCount = inFs.tellg() / chunkSize;
    LOG(INFO) << format("frame count = {}", kFrameCount);

#if 0
    // read image and show
    inFs.seekg(0, ios::beg);
    for (int i = 0; i < kFrameCount; ++i) {
        vector<char> raw(chunkSize);
        inFs.read(raw.data(), chunkSize);

        cv::Mat yuv(height, width, CV_8UC3);
        std::vector<cv::Mat> yuvChannels;
        cv::split(yuv, yuvChannels);
        yuvChannels[0].data = reinterpret_cast<unsigned char*>(raw.data());                  // Y
        yuvChannels[1].data = reinterpret_cast<unsigned char*>(raw.data()) + ySize;          // U
        yuvChannels[2].data = reinterpret_cast<unsigned char*>(raw.data()) + ySize + uSize;  // V
        cv::merge(yuvChannels, yuv);

        cv::Mat bgr;
        cvtColor(yuv, bgr, cv::COLOR_YUV2BGR);
        cv::imshow("YUV", yuv);
        cv::imshow("BGR", bgr);
        cv::waitKey(100);
    }
#endif

    // set parameters of x246
    x264_param_t param;
    x264_param_default(&param);
    param.i_log_level = X264_LOG_DEBUG;
    param.i_width = width;
    param.i_height = height;
    param.i_threads = X264_SYNC_LOOKAHEAD_AUTO;
    param.i_frame_total = 0;
    param.i_keyint_max = 10;
    param.i_bframe = 5;
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
    x264_param_apply_profile(&param, x264_profile_names[5]);

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
    inFs.seekg(0, ios::beg);
    for (int i = 0; i < kFrameCount; ++i) {
        LOG(INFO) << format("encoding frame [{}]", i);
        inFs.read(reinterpret_cast<char*>(inPic.img.plane[0]), ySize);  // Y
        inFs.read(reinterpret_cast<char*>(inPic.img.plane[1]), uSize);  // U
        inFs.read(reinterpret_cast<char*>(inPic.img.plane[2]), vSize);  // V

        inPic.i_pts = i;

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
    inFs.close();
    outFs.close();

    google::ShutdownGoogleLogging();
    return 0;
}