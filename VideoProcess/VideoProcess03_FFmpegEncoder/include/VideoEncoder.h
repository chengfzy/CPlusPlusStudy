#pragma once
#include <boost/filesystem.hpp>
#include <vector>
#include "ImageFormat.hpp"

class VideoEncoder {
  public:
    /**
     * @brief Constructor
     *
     */
    VideoEncoder() = default;

    /**
     * @brief Destructor
     *
     */
    ~VideoEncoder() = default;

  public:
    /**
     * @brief Initialization with image folder, image size and image format
     *
     * @param imageFolder   Image folder which contain YUV images with ".bin" extension
     * @param width         Image width
     * @param heigh         Image height
     * @param imageFormat   Image format(colorspace)
     */
    void init(const boost::filesystem::path& imageFolder, int width, int heigh,
              ImageFormat imageFormat = ImageFormat::None);

    /**
     * @brief Show image
     *
     * @param waitTime  Wait time, [ms]
     */
    void show(int waitTime = 100);

    /**
     * @brief Save YUV image to .jpg
     *
     * @param saveFolder    Save folder
     */
    void saveJpeg(const boost::filesystem::path& saveFolder = "./data/images");

    /**
     * @brief Encode image to H264 video
     *
     * @param saveFile      Save file name
     * @param maxFameCount  Max frame count for encoding, 0 for all
     */
    void encode(const boost::filesystem::path& saveFile = "./data/video.h264", int maxFameCount = 0);

  private:
    std::vector<boost::filesystem::path> imageFiles_;  // image files

    ImageFormat imageFormat_ = ImageFormat::None;  // image format
    int width_ = 0;                                // image width
    int height_ = 0;                               // image height
    int ySize_ = 0;                                // Y channel size
    int uSize_ = 0;                                // U channel size
    int vSize_ = 0;                                // V channel size
    int chunkSize_ = 0;                            // chunk size
};
