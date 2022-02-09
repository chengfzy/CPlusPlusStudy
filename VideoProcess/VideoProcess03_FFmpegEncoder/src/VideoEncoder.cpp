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
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/opt.h>
#include <turbojpeg.h>
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
            case ImageFormat::YUV422P:
                cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_Y422);
            case ImageFormat::YUYV422:
                cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_YUYV);
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

    // find encoder
    auto codec = avcodec_find_encoder(AVCodecID::AV_CODEC_ID_H264);
    CHECK(codec != nullptr) << format("cannot found video codec");

    // allocate contex
    auto context = avcodec_alloc_context3(codec);
    CHECK(context != nullptr) << format("cannot allocate video codec context");

    // allocate packet
    auto packet = av_packet_alloc();
    CHECK(packet != nullptr) << format("cannot allocate video packet");

    // set parameters
    const int fps = 30;  // FPS,  [Hz]
    context->width = width_;
    context->height = height_;
    context->time_base = AVRational{1, fps};
    context->framerate = AVRational{fps, 1};
    context->max_b_frames = 0;
    context->gop_size = 10;
    context->pix_fmt = AVPixelFormat::AV_PIX_FMT_YUV422P;
    if (codec->id == AVCodecID::AV_CODEC_ID_H264) {
        av_opt_set(context->priv_data, "preset", "fast", 0);
    }

    // open codec
    int ret = avcodec_open2(context, codec, nullptr);
    CHECK(ret >= 0) << format("could not open codec: {}", av_err2str(ret));

    // open save file

    // register
    av_register_all();
    // init context
    AVFormatContext* formatCtx;
    avformat_alloc_output_context2(&formatCtx, nullptr, nullptr, saveFile.string().c_str());
    AVOutputFormat* fmt = formatCtx->oformat;

    // open output file
    int ret = avio_open(&formatCtx->pb, saveFile.string().c_str(), AVIO_FLAG_WRITE);
    LOG_IF(ERROR, ret < 0) << format("cannot open file \"{}\" to save H264 video", saveFile.string());

    // create stream
    const int fps = 30;  // FPS,  [Hz]
    auto videoStream = avformat_new_stream(formatCtx, nullptr);
    videoStream->time_base.num = 1;
    videoStream->time_base.den = fps;

    // set parameters
    auto codeCtx = videoStream->codec;
    codeCtx->codec_id = fmt->video_codec;
    codeCtx->codec_type = AVMEDIA_TYPE_VIDEO;
    codeCtx->pix_fmt = AV_PIX_FMT_YUYV422;
    codeCtx->width = width_;
    codeCtx->height = height_;
    codeCtx->time_base.num = 1;
    codeCtx->time_base.den = fps;
    codeCtx->max_b_frames = 0;
    AVDictionary* param{nullptr};
    av_dict_set(&param, "preset", "medium", 0);
    av_dict_set(&param, "tune", "zerolatency", 0);
    // show some information
    av_dump_format(formatCtx, 0, saveFile.string().c_str(), 1);

    // find and open codec
    auto codec = avcodec_find_encoder(codeCtx->codec_id);
    CHECK(codec != nullptr) << "Cannot find encoder";
    ret = avcodec_open2(codeCtx, codec, &param);
    CHECK(ret >= 0) << format("failed to open encoder, {}", ret);

    // create picture size
    auto frame = av_frame_alloc();
    auto picSize = avpicture_get_size(codeCtx->pix_fmt, width_, height_);
    uint8_t* picBuf = (uint8_t*)av_malloc(picSize);
    avpicture_fill((AVPicture*)frame, picBuf, codeCtx->pix_fmt, width_, height_);

    // write file header
    avformat_write_header(formatCtx, nullptr);

    // encode to H264
    int frameCount = imageFiles_.size();
    if (maxFameCount > 0 && frameCount > maxFameCount) {
        frameCount = maxFameCount;
    }
    AVPacket packet;
    av_new_packet(&packet, picSize);
    int gotPicture{0};
    for (int i = 0; i < frameCount; ++i) {
        LOG(INFO) << format("encoding frame [{}/{}] {}", i, frameCount, imageFiles_[i].string());

        // read image
        frame->pts = i;
        string fileName = imageFiles_[i].string();
        fstream fs(fileName, ios::in | ios::binary);
        if (!fs.is_open()) {
            LOG(FATAL) << format("cannot open image file \"{}\"", fileName);
            continue;
        }
        if (imageFormat_ == ImageFormat::YUV422P) {
            fs.read(reinterpret_cast<char*>(frame->data[0]), ySize_);
            fs.read(reinterpret_cast<char*>(frame->data[1]), uSize_);
            fs.read(reinterpret_cast<char*>(frame->data[2]), vSize_);
        } else {
            fs.read(reinterpret_cast<char*>(frame->data[0]), chunkSize_);
        }
        fs.close();

        // encode
        ret = avcodec_encode_video2(codeCtx, &packet, frame, &gotPicture);
        CHECK(ret >= 0) << format("failed to encode, {}", ret);
        if (gotPicture == 1) {
            packet.stream_index = videoStream->index;
            ret = av_write_frame(formatCtx, &packet);
            av_free_packet(&packet);
        }
    }
    // flush encoder
    if (formatCtx->streams[0]->codec->codec->capabilities & AV_CODEC_CAP_DELAY) {
        while (true) {
            packet.data = nullptr;
            packet.size = 0;
            av_init_packet(&packet);
            ret = avcodec_encode_video2(formatCtx->streams[0]->codec, &packet, nullptr, &gotPicture);
            av_frame_free(nullptr);
            if (ret < 0) {
                break;
            }
            if (!gotPicture) {
                break;
            }
            LOG(INFO) << format("flush 1 frame");
            ret = av_write_frame(formatCtx, &packet);
            if (ret < 0) {
                break;
            }
        }
    }

    // write tailer
    av_write_trailer(formatCtx);

    // clean
    if (videoStream) {
        avcodec_close(videoStream->codec);
        av_free(frame);
        av_free(picBuf);
    }
    avio_close(formatCtx->pb);
    avformat_free_context(formatCtx);
}

#if 0
void VideoEncoder::encode(const boost::filesystem::path& saveFile, int maxFameCount) {
    LOG(INFO) << format("encode YUV image to video file \"{}\"", saveFile.string());

    // register
    av_register_all();
    // init context
    AVFormatContext* formatCtx;
    avformat_alloc_output_context2(&formatCtx, nullptr, nullptr, saveFile.string().c_str());
    AVOutputFormat* fmt = formatCtx->oformat;

    // open output file
    int ret = avio_open(&formatCtx->pb, saveFile.string().c_str(), AVIO_FLAG_WRITE);
    LOG_IF(ERROR, ret < 0) << format("cannot open file \"{}\" to save H264 video", saveFile.string());

    // create stream
    const int fps = 30;  // FPS,  [Hz]
    auto videoStream = avformat_new_stream(formatCtx, nullptr);
    videoStream->time_base.num = 1;
    videoStream->time_base.den = fps;

    // set parameters
    auto codeCtx = videoStream->codec;
    codeCtx->codec_id = fmt->video_codec;
    codeCtx->codec_type = AVMEDIA_TYPE_VIDEO;
    codeCtx->pix_fmt = AV_PIX_FMT_YUYV422;
    codeCtx->width = width_;
    codeCtx->height = height_;
    codeCtx->time_base.num = 1;
    codeCtx->time_base.den = fps;
    codeCtx->max_b_frames = 0;
    AVDictionary* param{nullptr};
    av_dict_set(&param, "preset", "medium", 0);
    av_dict_set(&param, "tune", "zerolatency", 0);
    // show some information
    av_dump_format(formatCtx, 0, saveFile.string().c_str(), 1);

    // find and open codec
    auto codec = avcodec_find_encoder(codeCtx->codec_id);
    CHECK(codec != nullptr) << "Cannot find encoder";
    ret = avcodec_open2(codeCtx, codec, &param);
    CHECK(ret >= 0) << format("failed to open encoder, {}", ret);

    // create picture size
    auto frame = av_frame_alloc();
    auto picSize = avpicture_get_size(codeCtx->pix_fmt, width_, height_);
    uint8_t* picBuf = (uint8_t*)av_malloc(picSize);
    avpicture_fill((AVPicture*)frame, picBuf, codeCtx->pix_fmt, width_, height_);

    // write file header
    avformat_write_header(formatCtx, nullptr);

    // encode to H264
    int frameCount = imageFiles_.size();
    if (maxFameCount > 0 && frameCount > maxFameCount) {
        frameCount = maxFameCount;
    }
    AVPacket packet;
    av_new_packet(&packet, picSize);
    int gotPicture{0};
    for (int i = 0; i < frameCount; ++i) {
        LOG(INFO) << format("encoding frame [{}/{}] {}", i, frameCount, imageFiles_[i].string());

        // read image
        frame->pts = i;
        string fileName = imageFiles_[i].string();
        fstream fs(fileName, ios::in | ios::binary);
        if (!fs.is_open()) {
            LOG(FATAL) << format("cannot open image file \"{}\"", fileName);
            continue;
        }
        if (imageFormat_ == ImageFormat::YUV422P) {
            fs.read(reinterpret_cast<char*>(frame->data[0]), ySize_);
            fs.read(reinterpret_cast<char*>(frame->data[1]), uSize_);
            fs.read(reinterpret_cast<char*>(frame->data[2]), vSize_);
        } else {
            fs.read(reinterpret_cast<char*>(frame->data[0]), chunkSize_);
        }
        fs.close();

        // encode
        ret = avcodec_encode_video2(codeCtx, &packet, frame, &gotPicture);
        CHECK(ret >= 0) << format("failed to encode, {}", ret);
        if (gotPicture == 1) {
            packet.stream_index = videoStream->index;
            ret = av_write_frame(formatCtx, &packet);
            av_free_packet(&packet);
        }
    }
    // flush encoder
    if (formatCtx->streams[0]->codec->codec->capabilities & AV_CODEC_CAP_DELAY) {
        while (true) {
            packet.data = nullptr;
            packet.size = 0;
            av_init_packet(&packet);
            ret = avcodec_encode_video2(formatCtx->streams[0]->codec, &packet, nullptr, &gotPicture);
            av_frame_free(nullptr);
            if (ret < 0) {
                break;
            }
            if (!gotPicture) {
                break;
            }
            LOG(INFO) << format("flush 1 frame");
            ret = av_write_frame(formatCtx, &packet);
            if (ret < 0) {
                break;
            }
        }
    }

    // write tailer
    av_write_trailer(formatCtx);

    // clean
    if (videoStream) {
        avcodec_close(videoStream->codec);
        av_free(frame);
        av_free(picBuf);
    }
    avio_close(formatCtx->pb);
    avformat_free_context(formatCtx);
}
#endif