#include "VideoDecoder.h"
#include <fmt/format.h>
#include <fmt/ostream.h>
#include <glog/logging.h>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace fmt;

/**
 * @brief Get the error string of FFmpeg, to avoid build error when using "av_err2str(errnum)" macro in C++
 *
 * @param errNum Error number
 * @return Error string
 */
string av_errstr(int errNum) {
    string str(AV_ERROR_MAX_STRING_SIZE, 0);
    av_strerror(errNum, str.data(), str.size());
    return str;
}

// Decode video file and save images to folder
void VideoDecoder::decode(const boost::filesystem::path& videoFile, const boost::filesystem::path& saveFolder) {
    LOG(INFO) << format("decode video: \"{}\"", videoFile.string());
    LOG(INFO) << format("save folder: \"{}\"", saveFolder.string());

    // create save directories if not exist
    boost::filesystem::create_directories(saveFolder);

    // find decoder
    // avcodec_register_all();
    auto codec = avcodec_find_decoder(AVCodecID::AV_CODEC_ID_H264);
    CHECK(codec != nullptr) << format("cannot found video codec");

    // init parser
    auto parser = av_parser_init(codec->id);
    CHECK(parser != nullptr) << format("cannot init video parser");

    // allocate packet
    auto packet = av_packet_alloc();
    CHECK(packet != nullptr) << format("cannot allocate video packet");

    // allocate contex
    auto context = avcodec_alloc_context3(codec);
    CHECK(context != nullptr) << format("cannot allocate video codec context");

    // open codec
    int ret = avcodec_open2(context, codec, nullptr);
    CHECK(ret >= 0) << format("could not open codec: {}", av_errstr(ret));

    // allocate frame
    auto frame = av_frame_alloc();
    CHECK(frame != nullptr) << format("cannot allocate video frame");

    // open video file
    ifstream fs(videoFile.string(), ios::in | ios::binary);
    CHECK(fs.is_open()) << format("cannot open file \"{}\" to save H264 video", videoFile.string());

    // set input buffer
    const int kBuffSize{4096};
    char buffer[kBuffSize + AV_INPUT_BUFFER_PADDING_SIZE];
    // set end of buffer to 0, this ensure that no overreading happens for damaged video stream
    memset(buffer + kBuffSize, 0, AV_INPUT_BUFFER_PADDING_SIZE);

    // decode
    while (!fs.eof()) {
        size_t dataSize = fs.readsome(buffer, kBuffSize);
        if (dataSize == 0) {
            break;
        }

        // use the parser to split the data into frames
        uint8_t* data = reinterpret_cast<uint8_t*>(buffer);
        while (dataSize > 0) {
            ret = av_parser_parse2(parser, context, &packet->data, &packet->size, data, dataSize, AV_NOPTS_VALUE,
                                   AV_NOPTS_VALUE, 0);
            if (ret < 0) {
                LOG(FATAL) << format("error while parsing: {}", av_errstr(ret));
                return;
            }
            data += ret;
            dataSize -= ret;

            if (packet->size > 0) {
                decodeFrame(context, frame, packet, saveFolder);
            }
        }
    }

    // flush the decoder
    decodeFrame(context, frame, nullptr, saveFolder);

    // free and close
    fs.close();
    av_parser_close(parser);
    avcodec_free_context(&context);
    av_frame_free(&frame);
    av_packet_free(&packet);
}

// Decode one frame
void VideoDecoder::decodeFrame(AVCodecContext* context, AVFrame* frame, AVPacket* packet,
                               const boost::filesystem::path& saveFolder) {
    // send packet to decoder
    int ret = avcodec_send_packet(context, packet);
    CHECK(ret >= 0) << format("could not send a packet for decoding: {}", av_errstr(ret));

    while (ret >= 0) {
        ret = avcodec_receive_frame(context, frame);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
            return;
        } else if (ret < 0) {
            LOG(FATAL) << format("could during decoding: {}", av_errstr(ret));
        }

        // set image size and format
        if (width_ == 0 || height_ == 0) {
            width_ = context->width;
            height_ = context->height;
            imageFormat_ = context->pix_fmt;
            switch (imageFormat_) {
                case AVPixelFormat::AV_PIX_FMT_YUV420P:
                    uWidth_ = width_ / 4;
                    vWidth_ = width_ / 4;
                    break;
                case AVPixelFormat::AV_PIX_FMT_YUV422P:
                    uWidth_ = width_ / 2;
                    vWidth_ = width_ / 2;
                    break;
                default:
                    LOG(FATAL) << format("unsupported image format {}", av_get_pix_fmt_name(imageFormat_));
                    break;
            }
            LOG(INFO) << format("video frame format = {}, frame size = {}", av_get_pix_fmt_name(imageFormat_), width_,
                                height_);
        }

        // check format and image size
        LOG_IF(FATAL, context->width != width_ || context->height != height_)
            << format("unmatched frame size, new: {}x{}, old: {}x{}", context->width, context->height, width_, height_);
        LOG_IF(FATAL, context->pix_fmt != imageFormat_)
            << format("unmatched frame format, new: {}, old: {}", av_get_pix_fmt_name(context->pix_fmt),
                      av_get_pix_fmt_name(imageFormat_));

        // save image to file
        LOG(INFO) << format("decoding frame [{}] size = {}x{}, format = {}", context->frame_number, context->width,
                            context->height, context->pix_fmt);
        boost::filesystem::path saveFile = saveFolder / format("{}.bin", context->frame_number);
        fstream fs(saveFile.string(), ios::out | ios::binary);
        if (!fs.is_open()) {
            LOG(ERROR) << fmt::format("cannot create file \"{}\"", saveFile.string());
        }
        // Y
        for (int i = 0; i < height_; ++i) {
            fs.write(reinterpret_cast<const char*>(frame->data[0] + i * frame->linesize[0]), width_);
        }
        // U
        for (int i = 0; i < height_; ++i) {
            fs.write(reinterpret_cast<const char*>(frame->data[1] + i * frame->linesize[1]), uWidth_);
        }
        // V
        for (int i = 0; i < height_; ++i) {
            fs.write(reinterpret_cast<const char*>(frame->data[2] + i * frame->linesize[2]), vWidth_);
        }
        fs.close();

        // show image
        show(saveFile, 10);
    }
}

// Show image
void VideoDecoder::show(const boost::filesystem::path& imageFile, int waitTime) {
    if (imageFormat_ != AVPixelFormat::AV_PIX_FMT_YUV422P) {
        LOG(ERROR) << format("unsupported image format {}", av_get_pix_fmt_name(imageFormat_));
        return;
    }

    // read image
    fstream fs(imageFile.string(), ios::in | ios::binary);
    if (!fs.is_open()) {
        LOG(FATAL) << format("cannot open image file \"{}\"", imageFile.string());
        return;
    }
    vector<unsigned char> raw = vector<unsigned char>(istreambuf_iterator<char>(fs), {});
    fs.close();

    // convert YUV422P to YUYV422
    int ySize = width_ * height_;
    int vSize = ySize / 2;
    int uSize = ySize / 2;
    int chunkSize = ySize + uSize + vSize;
    vector<unsigned char> yuyvData(chunkSize);
    unsigned char* pY = raw.data();
    unsigned char* pU = raw.data() + ySize;
    unsigned char* pV = raw.data() + ySize + uSize;
    unsigned char* pDst = yuyvData.data();
    for (int i = 0; i < uSize; ++i) {
        *(pDst++) = *(pY++);
        *(pDst++) = *(pU++);
        *(pDst++) = *(pY++);
        *(pDst++) = *(pV++);
    }

    // to BGR and show
    cv::Mat bgr;
    cv::Mat yuyv(height_, width_, CV_8UC2, yuyvData.data());
    cv::cvtColor(yuyv, bgr, cv::COLOR_YUV2BGR_YUYV);

    // resize if too large
    if (bgr.rows > 720 || bgr.cols > 720) {
        float ratio = 720.F / max(bgr.rows, bgr.cols);
        cv::resize(bgr, bgr, cv::Size(), ratio, ratio);
    }
    cv::imshow("BGR", bgr);
    cv::waitKey(waitTime);
}
