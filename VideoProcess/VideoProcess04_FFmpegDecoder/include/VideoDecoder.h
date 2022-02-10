#pragma once
#include <boost/filesystem.hpp>
#include <vector>
#include "ImageFormat.hpp"

#ifdef __cplusplus
extern "C" {
#endif
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/pixdesc.h>
#include <turbojpeg.h>
#ifdef __cplusplus
};
#endif

class VideoDecoder {
  public:
    /**
     * @brief Constructor
     *
     */
    VideoDecoder() = default;

    /**
     * @brief Destructor
     *
     */
    ~VideoDecoder() = default;

  public:
    /**
     * @brief Decode video file and save images to folder
     *
     * @param videoFile     Video file
     * @param saveFolder    Folder for saving images
     */
    void decode(const boost::filesystem::path& videoFile = "./data/video.264",
                const boost::filesystem::path& saveFolder = "./data/images");

  private:
    /**
     * @brief Decode one frame
     *
     * @param context     Video codec context
     * @param frame       Video frame
     * @param packet      Video packet
     * @param saveFolder  Save folder
     */
    void decodeFrame(AVCodecContext* context, AVFrame* frame, AVPacket* packet,
                     const boost::filesystem::path& saveFolder);

    /**
     * @brief Show image
     *
     * @param imageFile Image file
     * @param waitTime  Wait time, [ms]
     */
    void show(const boost::filesystem::path& imageFile, int waitTime = 100);

  private:
    AVPixelFormat imageFormat_ = AVPixelFormat::AV_PIX_FMT_NONE;  // image format
    int width_ = 0;                                               // image width
    int height_ = 0;                                              // image height
    int uWidth_ = 0;                                              // width for U channel
    int vWidth_ = 0;                                              // width for V channel
};
