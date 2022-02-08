#include "VideoEncoder.h"
#include <fmt/format.h>
#include <glog/logging.h>
#include <turbojpeg.h>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace fmt;

void VideoEncoder::init(const boost::filesystem::path& imageFolder, int width, int heigh, int csp) {
    using namespace boost;
    using namespace boost::filesystem;

    // set
    LOG(INFO) << format("image folder: {}", imageFolder.string());
    imageFiles_.clear();
    width_ = width;
    height_ = heigh;
    csp_ = csp;
    LOG(INFO) << format("image size = {}x{}", width_, height_);

    // calculate channel and chunk size
    switch (csp_) {
        case X264_CSP_I420:  // YUV420
            ySize_ = width_ * height_;
            uSize_ = ySize_ / 4;
            vSize_ = ySize_ / 4;
            break;
        case X264_CSP_I422:  // YUV422(YUV422 planar)
        case X264_CSP_YUYV:  // YUYV(YUV422 packed)
            ySize_ = width_ * height_;
            uSize_ = ySize_ / 2;
            vSize_ = ySize_ / 2;
            break;
        default:
            LOG(ERROR) << format("unsupported colorspace {}", csp_);
            break;
    }
    chunkSize_ = ySize_ + uSize_ + vSize_;
    LOG(INFO) << format("color space = {}, Y/U/V size = {}/{}/{}, chunk size = {}", csp_, ySize_, uSize_, vSize_,
                        chunkSize_);

    // find images
    directory_iterator itEnd;
    for (directory_iterator it(imageFolder); it != itEnd; ++it) {
        if (!is_directory(*it)) {
            imageFiles_.emplace_back(canonical(it->path()));
        }
    }

    // sort
    sort(imageFiles_.begin(), imageFiles_.end(), [](const path& p1, const path& p2) {
        auto id1 = lexical_cast<uint64_t>(replace_all_copy(trim_copy(p1.stem().string()), "yuv", ""));
        auto id2 = lexical_cast<uint64_t>(replace_all_copy(trim_copy(p2.stem().string()), "yuv", ""));
        return id1 < id2;
    });
    LOG(INFO) << format("found {} images", imageFiles_.size());

    // print
    // for (size_t i = 0; i < imageFiles_.size(); ++i) {
    //     cout << format("\t[{}/{}] {}", i, imageFiles_.size(), imageFiles_[i].string()) << endl;
    // }
}

void VideoEncoder::show(int waitTime) {
    // read image and show
    for (size_t i = 0; i < imageFiles_.size(); ++i) {
        string fileName = imageFiles_[i].string();
        fstream fs(fileName, ios::in | ios::binary);
        if (!fs.is_open()) {
            LOG(FATAL) << format("cannot open image file \"{}\"", fileName);
            continue;
        }
        vector<unsigned char> raw = vector<unsigned char>(istreambuf_iterator<char>(fs), {});
        fs.close();
        LOG(INFO) << format("[{}/{}] {}, data size = {}", i, imageFiles_.size(), imageFiles_[i].string(), raw.size());

        // to BGR and show
        cv::Mat yuv(height_, width_, CV_8UC2, raw.data());
        cv::Mat bgr;

        // split
        // cv::Mat yChannel(height_, width_, CV_8UC1, raw.data());
        // cv::Mat uChannel(height_, width_ / 2, CV_8UC1, raw.data() + ySize_);
        // cv::Mat vChannel(height_, width_ / 2, CV_8UC1, raw.data() + ySize_ + uSize_);
        // cv::imshow("Y", yChannel);
        // cv::imshow("U", uChannel);
        // cv::imshow("V", vChannel);
        // cv::waitKey();

        // cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_YUYV);
        cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_YUY2);
        cv::resize(bgr, bgr, cv::Size(), 0.5, 0.5);  // resize
        cv::imshow("BGR", bgr);
        cv::waitKey(waitTime);
    }
}

// save YUV image to .jpg
void VideoEncoder::saveJpeg(const boost::filesystem::path& saveFolder) {
    LOG(INFO) << format("convert YUV422 packed(YUYV) to .jpg to \"{}\"", saveFolder.string());
    // create save folder
    boost::filesystem::create_directories(saveFolder);

    // init compressor
    tjhandle compressor = tjInitCompress();
    int maxBufferSize = tjBufSize(width_, height_, TJSAMP_422);
    unsigned char* dst = new unsigned char[maxBufferSize];  // dest buffer
    unsigned long dstSize = maxBufferSize;                  // dest size

    for (int i = 0; i < imageFiles_.size(); ++i) {
        LOG(INFO) << format("encoding frame [{}]", i);

        // read image
        string inputFileName = imageFiles_[i].string();
        fstream inFs(inputFileName, ios::in | ios::binary);
        if (!inFs.is_open()) {
            LOG(FATAL) << format("cannot open image file \"{}\"", inputFileName);
            continue;
        }
        vector<unsigned char> raw = vector<unsigned char>(istreambuf_iterator<char>(inFs), {});
        inFs.close();

        // convert YUYV(YUV422 packed) to YUV(YUV422 planar)
        vector<unsigned char> yuvData(chunkSize_);
        unsigned char* pRaw = raw.data();
        unsigned char* pY = yuvData.data();
        unsigned char* pU = yuvData.data() + ySize_;
        unsigned char* pV = yuvData.data() + ySize_ + uSize_;
        for (int i = 0; i < uSize_; ++i) {
            *(pY++) = *(pRaw++);
            *(pU++) = *(pRaw++);
            *(pY++) = *(pRaw++);
            *(pV++) = *(pRaw++);
        }

        // compress
        if (tjCompressFromYUV(compressor, yuvData.data(), width_, 1, height_, TJSAMP_422, &dst, &dstSize, 95,
                              TJFLAG_FASTDCT | TJFLAG_NOREALLOC) != 0) {
            LOG(ERROR) << fmt::format("turbo jpeg compress error: {}", tjGetErrorStr2(compressor));
        }

        // write to file
        boost::filesystem::path saveFile = saveFolder / format("{}.jpg", i);
        fstream outFs(saveFile.string(), ios::out | ios::binary);
        if (!outFs.is_open()) {
            LOG(ERROR) << fmt::format("cannot create file \"{}\"", saveFile.string());
        }
        outFs.write(reinterpret_cast<const char*>(dst), dstSize);
        outFs.close();
    }
}

// save to YUV planar image
void VideoEncoder::saveYUVPlanar(const boost::filesystem::path& saveFolder) {
    LOG(INFO) << format("convert YUV422 packed(YUYV) to YUV422 planar to \"{}\"", saveFolder.string());
    // create save folder
    boost::filesystem::create_directories(saveFolder);

    for (int i = 0; i < imageFiles_.size(); ++i) {
        LOG(INFO) << format("encoding frame [{}]", i);

        // read image
        string inputFileName = imageFiles_[i].string();
        fstream inFs(inputFileName, ios::in | ios::binary);
        if (!inFs.is_open()) {
            LOG(FATAL) << format("cannot open image file \"{}\"", inputFileName);
            continue;
        }
        vector<unsigned char> raw = vector<unsigned char>(istreambuf_iterator<char>(inFs), {});
        inFs.close();

        // convert YUYV(YUV422 packed) to YUV(YUV422 planar)
        vector<unsigned char> yuvData(chunkSize_);
        unsigned char* pRaw = raw.data();
        unsigned char* pY = yuvData.data();
        unsigned char* pU = yuvData.data() + ySize_;
        unsigned char* pV = yuvData.data() + ySize_ + uSize_;
        for (int i = 0; i < uSize_; ++i) {
            *(pY++) = *(pRaw++);
            *(pU++) = *(pRaw++);
            *(pY++) = *(pRaw++);
            *(pV++) = *(pRaw++);
        }

        // write to file
        boost::filesystem::path saveFile = saveFolder / format("{}.bin", i);
        fstream outFs(saveFile.string(), ios::out | ios::binary);
        if (!outFs.is_open()) {
            LOG(ERROR) << fmt::format("cannot create file \"{}\"", saveFile.string());
        }
        outFs.write(reinterpret_cast<const char*>(yuvData.data()), yuvData.size());
        outFs.close();
    }
}

void VideoEncoder::encode(const boost::filesystem::path& saveFile) {
    LOG(INFO) << format("encode YUV image to video file \"{}\"", saveFile.string());

    // set parameters of x246
    x264_param_t param;
    x264_param_default(&param);
    param.i_log_level = X264_LOG_DEBUG;
    param.i_width = width_;
    param.i_height = height_;
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
    param.i_csp = csp_;
    // x264_param_apply_profile(&param, x264_profile_names[5]);

    // open yuv and h264 file
    ofstream outFs(saveFile.string(), ios::binary);
    CHECK(outFs.is_open()) << format("cannot open \"{}\" to save H264 video", saveFile.string());

    // alloc memory for input image
    x264_picture_t inPic;
    x264_picture_alloc(&inPic, csp_, width_, height_);
    // out image
    x264_picture_t outPic;
    x264_picture_init(&outPic);
    x264_nal_t* nal = nullptr;
    int iNal{0};

    // init encoder
    x264_t* handle = x264_encoder_open(&param);
    CHECK(handle != nullptr) << "cannot init encoder";

    // encode to H264
    for (int i = 0; i < imageFiles_.size(); ++i) {
        LOG(INFO) << format("encoding frame [{}] {}", i, imageFiles_[i].string());

        // read image
        inPic.i_pts = i;
        string fileName = imageFiles_[i].string();
        fstream fs(fileName, ios::in | ios::binary);
        if (!fs.is_open()) {
            LOG(FATAL) << format("cannot open image file \"{}\"", fileName);
            continue;
        }
        // vector<unsigned char> raw = vector<unsigned char>(istreambuf_iterator<char>(fs), {});
        // vector<unsigned char> raw = vector<unsigned char>(istreambuf_iterator<char>(fs), {});
        // inPic.img.plane[0] =

        if (csp_ == X264_CSP_I422) {
            fs.read(reinterpret_cast<char*>(inPic.img.plane[0]), ySize_);
            fs.read(reinterpret_cast<char*>(inPic.img.plane[1]), uSize_);
            fs.read(reinterpret_cast<char*>(inPic.img.plane[2]), vSize_);
        } else {
            fs.read(reinterpret_cast<char*>(inPic.img.plane[0]), chunkSize_);
        }
        fs.close();

        // convert YUYV(YUV422 packed) to YUV(YUV422 planar)
        // inPic.i_pts = i;
        // unsigned char* pRaw = raw.data();
        // uint8_t* pY = inPic.img.plane[0];
        // uint8_t* pU = inPic.img.plane[1];
        // uint8_t* pV = inPic.img.plane[2];
        // for (int m = 0; m < uSize; ++m) {
        //     *(pY++) = *(pRaw++);
        //     *(pU++) = *(pRaw++);
        //     *(pY++) = *(pRaw++);
        //     *(pV++) = *(pRaw++);
        // }

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
}