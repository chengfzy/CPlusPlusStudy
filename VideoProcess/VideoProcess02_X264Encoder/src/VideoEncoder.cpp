#include "VideoEncoder.h"
#include <fmt/format.h>
#include <glog/logging.h>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

#ifdef __cplusplus
extern "C" {
#endif
#include <turbojpeg.h>
#include <x264.h>
#ifdef __cplusplus
};
#endif

using namespace std;
using namespace fmt;

// Initialization with image folder, image size and image format
void VideoEncoder::init(const boost::filesystem::path& imageFolder, int width, int heigh, ImageFormat imageFormat) {
    using namespace boost;
    using namespace boost::filesystem;

    // set
    LOG(INFO) << format("image folder: {}", imageFolder.string());
    imageFiles_.clear();
    imageFormat_ = imageFormat;
    width_ = width;
    height_ = heigh;
    LOG(INFO) << format("image format = {}, image size = {}x{}", imageFormat_, width_, height_);

    // calculate channel and chunk size
    switch (imageFormat_) {
        case ImageFormat::YUV420P:  // YUV 4:2:0 planar
            ySize_ = width_ * height_;
            uSize_ = ySize_ / 4;
            vSize_ = ySize_ / 4;
            break;
        case ImageFormat::YUYV422:  // YUYV 4:2:2 packed
        case ImageFormat::YUV422P:  // YUV 4:2:2 planar
            ySize_ = width_ * height_;
            uSize_ = ySize_ / 2;
            vSize_ = ySize_ / 2;
            break;
        default:
            LOG(ERROR) << format("unsupported image format {}", imageFormat_);
            break;
    }
    chunkSize_ = ySize_ + uSize_ + vSize_;
    LOG(INFO) << format("Y/U/V size = {}/{}/{}, chunk size = {}", ySize_, uSize_, vSize_, chunkSize_);

    // find images
    directory_iterator itEnd;
    for (directory_iterator it(imageFolder); it != itEnd; ++it) {
        if (is_regular_file(*it)) {
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
}

// Show image
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

        cv::Mat yuv(height_, width_, CV_8UC2, raw.data());
        // split
        // cv::Mat yChannel(height_, width_, CV_8UC1, raw.data());
        // cv::Mat uChannel(height_, width_ / 2, CV_8UC1, raw.data() + ySize_);
        // cv::Mat vChannel(height_, width_ / 2, CV_8UC1, raw.data() + ySize_ + uSize_);
        // cv::imshow("Y", yChannel);
        // cv::imshow("U", uChannel);
        // cv::imshow("V", vChannel);
        // cv::waitKey();

        // to BGR and show
        cv::Mat bgr;
        switch (imageFormat_) {
            case ImageFormat::YUV420P:
                cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_I420);
                break;
            case ImageFormat::YUV422P: {
                vector<unsigned char> yuyvData(chunkSize_);
                unsigned char* pY = raw.data();
                unsigned char* pU = raw.data() + ySize_;
                unsigned char* pV = raw.data() + ySize_ + uSize_;
                unsigned char* pDst = yuyvData.data();
                for (int i = 0; i < uSize_; ++i) {
                    *(pDst++) = *(pY++);
                    *(pDst++) = *(pU++);
                    *(pDst++) = *(pY++);
                    *(pDst++) = *(pV++);
                }
                cv::Mat yuyv(height_, width_, CV_8UC2, yuyvData.data());
                cv::cvtColor(yuyv, bgr, cv::COLOR_YUV2BGR_YUYV);
                break;
            }
            case ImageFormat::YUYV422:
                cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_YUYV);
                break;
            default:
                LOG(ERROR) << format("unsupported image format {}", imageFormat_);
                break;
        }
        // resize if too large
        if (bgr.rows > 720 || bgr.cols > 720) {
            float ratio = 720.F / max(bgr.rows, bgr.cols);
            cv::resize(bgr, bgr, cv::Size(), ratio, ratio);
        }
        cv::imshow("BGR", bgr);
        cv::waitKey(waitTime);
    }
}

// save YUV image to .jpg
void VideoEncoder::saveJpeg(const boost::filesystem::path& saveFolder) {
    if (imageFormat_ != ImageFormat::YUYV422) {
        LOG(ERROR) << format("unsupported image format {}", imageFormat_);
        return;
    }
    LOG(INFO) << format("convert YUV image to .jpg, and save to \"{}\"", saveFolder.string());
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

// Encode image to H264 video
void VideoEncoder::encode(const boost::filesystem::path& saveFile, int maxFameCount) {
    LOG(INFO) << format("encode YUV image to video file \"{}\"", saveFile.string());

    // open output h264 file
    ofstream outFs(saveFile.string(), ios::binary);
    CHECK(outFs.is_open()) << format("cannot open file \"{}\" to save H264 video", saveFile.string());

    // get color space
    int csp{X264_CSP_NONE};
    switch (imageFormat_) {
        case ImageFormat::YUV420P:
            csp = X264_CSP_I420;
            break;
        case ImageFormat::YUV422P:
            csp = X264_CSP_I422;
            break;
        case ImageFormat::YUYV422:
            csp = X264_CSP_YUYV;
            break;
        default:
            LOG(ERROR) << format("unsupported image format {}", imageFormat_);
            return;
            break;
    }

    // set parameters of x246
    x264_param_t param;
    x264_param_default_preset(&param, "fast", "zerolatency");
    const int fps = 30;  // FPS,  [Hz]
    param.i_csp = csp;
    param.i_width = width_;
    param.i_height = height_;
    param.i_log_level = X264_LOG_DEBUG;  // log
    param.i_threads = 1;                 // 1 thread
    param.i_frame_total = 0;
    param.i_bframe = 0;  // no b frame
    param.i_keyint_max = fps * 2;
    param.i_keyint_min = fps * 2;
    param.i_fps_den = 1;
    param.i_fps_num = fps;
    param.i_timebase_den = param.i_fps_num;
    param.i_timebase_num = param.i_fps_den;
    // use QP for quality controlling, [0, 51], 0 mean lossess, 51 mean more worst
    param.rc.i_rc_method = X264_RC_CQP;
    param.rc.i_qp_constant = 0;
    // or use CRF(Constant Rate Factor) for quality controlling, [0, 51], 0 mean lossess, 51 mean more worst
    // param.rc.f_rf_constant = 0;

    // alloc memory for input image
    x264_picture_t inPic;
    int ret = x264_picture_alloc(&inPic, csp, width_, height_);
    // out image
    x264_picture_t outPic;
    x264_picture_init(&outPic);
    x264_nal_t* nal = nullptr;
    int iNal{0};

    // init encoder
    x264_t* handle = x264_encoder_open(&param);
    CHECK(handle != nullptr) << "cannot init encoder";

    // encode to H264
    int frameCount = imageFiles_.size();
    if (maxFameCount > 0 && frameCount > maxFameCount) {
        frameCount = maxFameCount;
    }
    for (int i = 0; i < frameCount; ++i) {
        LOG(INFO) << format("encoding frame [{}/{}] {}", i, frameCount, imageFiles_[i].string());

        // read image
        inPic.i_pts = i;
        string fileName = imageFiles_[i].string();
        fstream fs(fileName, ios::in | ios::binary);
        if (!fs.is_open()) {
            LOG(FATAL) << format("cannot open image file \"{}\"", fileName);
            continue;
        }
        if (imageFormat_ == ImageFormat::YUV422P) {
            fs.read(reinterpret_cast<char*>(inPic.img.plane[0]), ySize_);
            fs.read(reinterpret_cast<char*>(inPic.img.plane[1]), uSize_);
            fs.read(reinterpret_cast<char*>(inPic.img.plane[2]), vSize_);
        } else {
            fs.read(reinterpret_cast<char*>(inPic.img.plane[0]), chunkSize_);
        }
        fs.close();

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