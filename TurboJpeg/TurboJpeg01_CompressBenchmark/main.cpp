/**
 * @brief Benchmark code for YUYV image compression
 *
 * 1. Convert YUYV to BGR using OpenCV, then write to file
 * 2. Convert YUYV to BGR using OpenCV, then compress BGR image using turbo-jpeg, finally write to file
 * 3. Compress YUYV to JPG using jpeg-lib, then write to file
 * 4. Convert YUYV(YUV422 Packed) to YUV(YUV422 Planar), then compress using TurboJpeg, and finally write to file
 * 5. Convert YUYV to BGR, the conversion is calculated every time, then write to file
 * 6. Convert YUYV to BGR, and calculate the YUV tot BGR map table at first, then write to file
 */

#include <fmt/format.h>
#include <glog/logging.h>
#include <jpeglib.h>
#include <sched.h>
#include <turbojpeg.h>
#include <array>
#include <chrono>
#include <fstream>
#include <iostream>
#include <numeric>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace std::chrono;
using namespace cv;

int main(int argc, char* argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;

    // set CPU affinity
    cpu_set_t mask;
    CPU_ZERO(&mask);
    CPU_SET(0, &mask);
    if (sched_setaffinity(0, sizeof(mask), &mask) < 0) {
        LOG(FATAL) << ("set CPU affinity failed");
        return -1;
    }

    // read raw YUYV buffer(YUV422 Packed)
    string fileName = "./data/yuyv.bin";  // yuyv image file
    int width = 1920;                     // image width
    int height = 1080;                    // image height
    fstream fs(fileName, ios::in | ios::binary);
    if (!fs.is_open()) {
        LOG(FATAL) << fmt::format("cannot open file \"{}\"", fileName);
        return 0;
    }
    vector<unsigned char> raw = vector<unsigned char>(istreambuf_iterator<char>(fs), {});
    fs.close();

    constexpr size_t kRepeatNum{1000};  // repeat num
    constexpr size_t kAlgNum{6};        // algorithm num
    vector<string> saveFiles{"./data/OpenCV.png",    "./data/OpenCVTurboJpeg.jpg", "./data/jpeg.jpg",
                             "./data/TurboJpeg.jpg", "./data/YuyvBgr1.bin",        "./data/YuyvBgr2.bin"};
    vector<array<int64_t, kAlgNum>> usedTime(kRepeatNum);
    vector<unsigned char> yuvData;  // YUV data for turbojpeg

    // allocate buffer size before compress for method 03
    int maxBufferSize = tjBufSize(width, height, TJSAMP_422);
    unsigned char* dest3 = new unsigned char[maxBufferSize];  // dest buffer
    unsigned long destSize3 = maxBufferSize;                  // dest size

    // dest size for method 04
    vector<unsigned char> dst4(width * height * 3);

    // dest size for method 04
    vector<unsigned char> dst5(width * height * 3);

    // calculate the map table at first
    // vector<vector<vector<vector<unsigned char>>>> yuv2bgrMap(256);
    unsigned char**** yuv2bgrMap = new unsigned char***[256];  //[U][V][Y][RGB]
    for (int i = 0; i < 256; ++i) {                            // U
        // yuv2bgrMap[i].resize(256);
        yuv2bgrMap[i] = new unsigned char**[256];
        for (int j = 0; j < 256; ++j) {  // V
            // yuv2bgrMap[i][j].resize(256);
            yuv2bgrMap[i][j] = new unsigned char*[256];
            for (int k = 0; k < 256; ++k) {  // Y
                // yuv2bgrMap[i][j][k].resize(3);
                yuv2bgrMap[i][j][k] = new unsigned char[3];
                int r = k + (1.370705 * (j - 128));
                int g = k - (0.698001 * (j - 128)) - (0.337633 * (i - 128));
                int b = k + (1.732446 * (i - 128));
                r = max(0, min(255, r)) * 220. / 256;
                g = max(0, min(255, g)) * 220. / 256;
                b = max(0, min(255, b)) * 220. / 256;
                yuv2bgrMap[i][j][k][0] = b;  // B
                yuv2bgrMap[i][j][k][1] = g;  // G
                yuv2bgrMap[i][j][k][2] = r;  // R
            }
        }
    }

    // test for each method
    for (size_t i = 0; i < kRepeatNum; i++) {
        cout << fmt::format("[{}/{}]", i, kRepeatNum);

        // 1. Convert YUYV to BGR using OpenCV, then write to file
        {
            auto t0 = steady_clock::now();
            Mat yuv(height, width, CV_8UC2, raw.data());
            Mat bgr;
            cvtColor(yuv, bgr, COLOR_YUV2BGR_YUYV);
            imwrite(saveFiles[0], bgr);
            auto t1 = steady_clock::now();
            auto dt = duration_cast<microseconds>(t1 - t0).count();
            cout << fmt::format("OpenCV = {:.2f} ms", dt / 1000.);
            usedTime[i][0] = dt;
        }

        // 2. Convert YUYV to BGR using OpenCV, then compress BGR image using turbo-jpeg, finally write to file
        {
            // init compressor
            tjhandle compressor = tjInitCompress();

            auto t0 = steady_clock::now();
            Mat yuv(height, width, CV_8UC2, raw.data());
            Mat bgr;
            cvtColor(yuv, bgr, COLOR_YUV2BGR_YUYV);
            auto t2 = steady_clock::now();

            // compress BGR image using turbojpeg
            unsigned char* dest = nullptr;  // dest buffer
            unsigned long destSize{0};      // dest size
            if (tjCompress2(compressor, bgr.data, width, 0, height, TJPF_BGR, &dest, &destSize, TJSAMP_444, 95,
                            TJFLAG_FASTDCT) != 0) {
                LOG(ERROR) << fmt::format("turbo jpeg compress error: {}", tjGetErrorStr2(compressor));
            }
            // write to file
            fstream fs(saveFiles[1], ios::out | ios::binary);
            if (!fs.is_open()) {
                LOG(ERROR) << fmt::format("cannot create file \"{}\"", saveFiles[1]);
            }
            fs.write(reinterpret_cast<const char*>(dest), destSize);
            fs.close();

            // release turbo jpeg data buffer
            tjFree(dest);

            auto t1 = steady_clock::now();
            auto dt = duration_cast<microseconds>(t1 - t0).count();   // all
            auto dt2 = duration_cast<microseconds>(t2 - t0).count();  // convert YUV=>BGR
            cout << fmt::format(", OpenCV+TurboJpeg = {:.2f}/{:.2f} ms", dt2 / 1000., dt / 1000.);
            usedTime[i][1] = dt;

            // destory compressor
            tjDestroy(compressor);
        }

        // 3. Compress YUYV to JPG using jpeg-lib, then write to file
        {
            auto t0 = steady_clock::now();
            unsigned char* dest = nullptr;  // dest buffer
            unsigned long destSize{0};      // dest size

            jpeg_compress_struct cinfo;
            jpeg_error_mgr jerr;
            JSAMPROW rowPtr[1];
            cinfo.err = jpeg_std_error(&jerr);
            jpeg_create_compress(&cinfo);
            jpeg_mem_dest(&cinfo, &dest, &destSize);
            cinfo.image_width = width & -1;
            cinfo.image_height = height & -1;
            cinfo.input_components = 3;
            cinfo.in_color_space = JCS_YCbCr;
            cinfo.dct_method = JDCT_IFAST;

            jpeg_set_defaults(&cinfo);
            jpeg_set_quality(&cinfo, 95, TRUE);
            jpeg_start_compress(&cinfo, TRUE);

            vector<uint8_t> tmprowbuf(width * 3);
            JSAMPROW row_pointer[1];
            row_pointer[0] = &tmprowbuf[0];
            while (cinfo.next_scanline < cinfo.image_height) {
                unsigned i, j;
                unsigned offset = cinfo.next_scanline * cinfo.image_width * 2;  // offset to the correct row
                for (i = 0, j = 0; i < cinfo.image_width * 2; i += 4, j += 6) {
                    // input strides by 4 bytes, output strides by 6 (2 pixels)
                    tmprowbuf[j + 0] = raw.data()[offset + i + 0];  // Y (unique to this pixel)
                    tmprowbuf[j + 1] = raw.data()[offset + i + 1];  // U (shared between pixels)
                    tmprowbuf[j + 2] = raw.data()[offset + i + 3];  // V (shared between pixels)
                    tmprowbuf[j + 3] = raw.data()[offset + i + 2];  // Y (unique to this pixel)
                    tmprowbuf[j + 4] = raw.data()[offset + i + 1];  // U (shared between pixels)
                    tmprowbuf[j + 5] = raw.data()[offset + i + 3];  // V (shared between pixels)
                }
                jpeg_write_scanlines(&cinfo, row_pointer, 1);
            }

            jpeg_finish_compress(&cinfo);

            // write to file
            fstream fs(saveFiles[2], ios::out | ios::binary);
            if (!fs.is_open()) {
                LOG(ERROR) << fmt::format("cannot create file \"{}\"", saveFiles[2]);
            }
            fs.write(reinterpret_cast<const char*>(dest), destSize);
            fs.close();

            // release data
            jpeg_destroy_compress(&cinfo);

            // record time
            auto t1 = steady_clock::now();
            auto dt = duration_cast<microseconds>(t1 - t0).count();
            cout << fmt::format(", JPEG = {:.2f} ms", dt / 1000.);
            usedTime[i][2] = dt;
        }

        // 4. convert YUYV(YUV422 Packed) to YUV(YUV422 Planar), then compress using TurboJpeg, and write to file
        {
            // init compressor
            tjhandle compressor = tjInitCompress();

            auto t0 = steady_clock::now();
            int length = 2 * width * height;
            if (yuvData.size() != length) {
                yuvData.resize(length);
            }
            unsigned char* pY = yuvData.data();
            unsigned char* pU = yuvData.data() + width * height;
            unsigned char* pV = yuvData.data() + width * height * 3 / 2;
            unsigned char* pRaw = raw.data();
            for (int i = 0; i < length / 4; ++i) {
                *(pY++) = *(pRaw++);
                *(pU++) = *(pRaw++);
                *(pY++) = *(pRaw++);
                *(pV++) = *(pRaw++);
            }

            // compress
            if (tjCompressFromYUV(compressor, yuvData.data(), width, 1, height, TJSAMP_422, &dest3, &destSize3, 95,
                                  TJFLAG_FASTDCT | TJFLAG_NOREALLOC) != 0) {
                LOG(ERROR) << fmt::format("turbo jpeg compress error: {}", tjGetErrorStr2(compressor));
            }
            auto t2 = steady_clock::now();

            // write to file
            fstream fs(saveFiles[3], ios::out | ios::binary);
            if (!fs.is_open()) {
                LOG(ERROR) << fmt::format("cannot create file \"{}\"", saveFiles[3]);
            }
            fs.write(reinterpret_cast<const char*>(dest3), destSize3);
            fs.close();
            auto t1 = steady_clock::now();

            // decode using OpenCV
            cv::Mat buffer(1, destSize3, CV_8UC1, (void*)dest3);
            cv::Mat image = cv::imdecode(buffer, cv::IMREAD_UNCHANGED);
            auto t3 = steady_clock::now();

            // decode using TurboJpeg
            unsigned char* decodeBuffer = new unsigned char[width * height * 3];
            tjhandle deCompressor = tjInitDecompress();
            if (tjDecompress2(deCompressor, dest3, destSize3, decodeBuffer, width, 0, height, TJPF_RGB,
                              TJFLAG_FASTDCT) != 0) {
                LOG(ERROR) << fmt::format("turbo jpeg decompress error: {}", tjGetErrorStr2(compressor));
            }
            auto t4 = steady_clock::now();

            // record time
            auto dt = duration_cast<microseconds>(t1 - t0).count();   // all
            auto dt2 = duration_cast<microseconds>(t2 - t0).count();  // compress
            auto dt3 = duration_cast<microseconds>(t3 - t1).count();  // decode using OpenCV
            auto dt4 = duration_cast<microseconds>(t4 - t3).count();  // decode using TurboJPEG
            cout << fmt::format(", TurboJpeg = {:.2f}/{:.2f}/{:.2f}/{:.2f} ms", dt2 / 1000., dt3 / 1000., dt4 / 1000.,
                                dt / 1000.);
            usedTime[i][3] = dt;

            // destory compressor
            tjDestroy(compressor);
            tjDestroy(deCompressor);
            delete[] decodeBuffer;
        }

        // 5. Convert YUYV to BGR, the conversion is calculated every time, then write to file
        {
            auto t0 = steady_clock::now();
            int length = 3 * width * height;
            if (dst4.size() != length) {
                dst4.resize(length);
            }
            unsigned char* pRaw = raw.data();
            unsigned char* pDst = dst4.data();
            for (int i = 0; i < width * height / 2; ++i) {
                int y0 = *(pRaw++);
                int u = *(pRaw++);
                int y1 = *(pRaw++);
                int v = *(pRaw++);

                int yy = 1220542 * max(0, y0 - 16);
                int vv = v - 128;
                int uu = u - 128;
                int c = 1 << 19;

                // convert to bgr for Y0, U, V
                // int r = 1.164 * (y0 - 16) + 1.596 * (v - 128);
                // int g = 1.164 * (y0 - 16) - 0.813 * (v - 128) - 0.391 * (u - 128);
                // int b = 1.164 * (y0 - 16) + 2.018 * (u - 128);
                int r = max(0, min(255, (yy + 1673527 * vv + c) >> 20));
                int g = max(0, min(255, (yy - 852492 * vv - 409993 * uu + c) >> 20));
                int b = max(0, min(255, (yy + 2116026 * uu + c) >> 20));
                // int r = y0 + (1.370705 * (v - 128));
                // int g = y0 - (0.698001 * (v - 128)) - (0.337633 * (u - 128));
                // int b = y0 + (1.732446 * (u - 128));
                // r = max(0, min(255, r)) * 220 / 256;
                // g = max(0, min(255, g)) * 220 / 256;
                // b = max(0, min(255, b)) * 220 / 256;

                *(pDst++) = b;
                *(pDst++) = g;
                *(pDst++) = r;

                // convert to bgr for Y1, U, V
                yy = 1220542 * max(0, y1 - 16);
                r = max(0, min(255, (yy + 1673527 * vv + c) >> 20));
                g = max(0, min(255, (yy - 852492 * vv - 409993 * uu + c) >> 20));
                b = max(0, min(255, (yy + 2116026 * uu + c) >> 20));
                // r = y1 + (1.370705 * (v - 128));
                // g = y1 - (0.698001 * (v - 128)) - (0.337633 * (u - 128));
                // b = y1 + (1.732446 * (u - 128));
                // r = max(0, min(255, r)) * 220 / 256;
                // g = max(0, min(255, g)) * 220 / 256;
                // b = max(0, min(255, b)) * 220 / 256;

                *(pDst++) = b;
                *(pDst++) = g;
                *(pDst++) = r;
            }
            auto t2 = steady_clock::now();

            // convert to Mat and show
            // Mat img(height, width, CV_8UC3, dst4.data());
            // imshow("YUYV2BG", img);
            // waitKey();

            // write to file
            fstream fs(saveFiles[4], ios::out | ios::binary);
            if (!fs.is_open()) {
                LOG(ERROR) << fmt::format("cannot create file \"{}\"", saveFiles[4]);
            }
            fs.write(reinterpret_cast<const char*>(dst4.data()), dst4.size());
            fs.close();

            // record time
            auto t1 = steady_clock::now();
            auto dt1 = duration_cast<microseconds>(t1 - t0).count();  // all
            auto dt2 = duration_cast<microseconds>(t2 - t0).count();  // compress
            cout << fmt::format(", YUYV2BGR = {:.2f}/{:.2f} ms", dt2 / 1000., dt1 / 1000.);
            usedTime[i][4] = dt1;
        }

        // 6. Convert YUYV to BGR, and calculate the YUV tot BGR map table at first, then write to file
        {
            auto t0 = steady_clock::now();
            int length = 3 * width * height;
            if (dst5.size() != length) {
                dst5.resize(length);
            }
            unsigned char* pRaw = raw.data();
            unsigned char* pDst = dst5.data();
            for (int i = 0; i < width * height / 2; ++i) {
                unsigned char y0 = *(pRaw++);
                unsigned char u = *(pRaw++);
                unsigned char y1 = *(pRaw++);
                unsigned char v = *(pRaw++);

                unsigned char** pD1 = yuv2bgrMap[u][v];
                unsigned char* pD = pD1[y0];
                *(pDst++) = pD[0];
                *(pDst++) = pD[1];
                *(pDst++) = pD[2];
                pD = pD1[y1];
                *(pDst++) = pD[0];
                *(pDst++) = pD[1];
                *(pDst++) = pD[2];
            }
            auto t2 = steady_clock::now();

            // convert to Mat and show
            // Mat img(height, width, CV_8UC3, dst5.data());
            // imshow("YUYV2BG with Table", img);
            // waitKey();

            // write to file
            fstream fs(saveFiles[5], ios::out | ios::binary);
            if (!fs.is_open()) {
                LOG(ERROR) << fmt::format("cannot create file \"{}\"", saveFiles[5]);
            }
            fs.write(reinterpret_cast<const char*>(dst5.data()), dst5.size());
            fs.close();

            // record time
            auto t1 = steady_clock::now();
            auto dt1 = duration_cast<microseconds>(t1 - t0).count();  // YUYV => BGR
            auto dt2 = duration_cast<microseconds>(t2 - t0).count();
            cout << fmt::format(", YUYV2BGR with Table = {:.2f}/{:.2f} ms", dt2 / 1000., dt1 / 1000.) << endl;
            usedTime[i][5] = dt1;
        }
    }

    // free buffer
    delete[] dest3;
    for (int i = 0; i < 256; ++i) {
        for (int j = 0; j < 256; ++j) {
            for (int k = 0; k < 256; ++k) {
                delete[] yuv2bgrMap[i][j][k];
            }
            delete[] yuv2bgrMap[i][j];
        }
        delete[] yuv2bgrMap[i];
    }
    delete[] yuv2bgrMap;

    // calculate the average time
    array<double, kAlgNum> averageTime;
    for (size_t i = 0; i < kAlgNum; ++i) {
        averageTime[i] = accumulate(usedTime.begin(), usedTime.end(), 0.,
                                    [&](const int64_t& a, const array<int64_t, kAlgNum>& v) { return a + v[i]; }) /
                         kRepeatNum;
    }
    cout << endl
         << fmt::format(
                "Average: OpenCV = {:.2f} ms, OpenCV+TurboJpeg = {:.2f} ms, JPEG = {:.2f} ms, TurboJpeg = {:.2f} ms, "
                "YUYV2BGR = {:.2f} ms, YUYV2BGR with Table = {:.2f} ms",
                averageTime[0] / 1000., averageTime[1] / 1000., averageTime[2] / 1000., averageTime[3] / 1000.,
                averageTime[4] / 1000., averageTime[5] / 1000.)
         << endl;

    google::ShutdownGoogleLogging();
    return 0;
}