#pragma once
#include <boost/filesystem.hpp>
#include <vector>

#include <x264.h>

class VideoEncoder {
  public:
    VideoEncoder() = default;

    ~VideoEncoder() = default;

  public:
    void init(const boost::filesystem::path& imageFolder, int width, int heigh, int csp = X264_CSP_NONE);

    void show(int waitTime = 100);

    // save YUV image to .jpg
    void saveJpeg(const boost::filesystem::path& saveFolder = "./data/images");

    // save to YUV planar image
    void saveYUVPlanar(const boost::filesystem::path& saveFolder = "./data/YUVPlanar");

    void encode(const boost::filesystem::path& saveFile = "./data/video.h264");

  private:
    std::vector<boost::filesystem::path> imageFiles_;  // image files

    int width_ = 0;      // image width
    int height_ = 0;     // image height
    int csp_ = 0;        // colorspace type
    int ySize_ = 0;      // Y channel size
    int uSize_ = 0;      // U channel size
    int vSize_ = 0;      // V channel size
    int chunkSize_ = 0;  // chunk size
};