#include <fmt/ostream.h>
#include <boost/asio.hpp>
#include <common/common.hpp>

using namespace fmt;
using namespace common;
using namespace boost::asio;

int main(int argc, const char* argv[]) {
    initLog(argc, argv);

    // function to obtain a string
    auto dayTimeStrFunc = []() {
        static int count{0};
        return format("count = {}", count++);
    };

    try {
        io_context io;
        ip::udp::socket socket(io, ip::udp::endpoint(ip::udp::v4(), 1234));
        while (true) {
            std::array<char, 1> recvBuf;
            ip::udp::endpoint remoteEndPoint;
            socket.receive_from(buffer(recvBuf), remoteEndPoint);
            LOG(INFO) << format("remote endpoint: {}", fmt::streamed(remoteEndPoint));

            auto message = dayTimeStrFunc();
            boost::system::error_code ec;
            socket.send_to(buffer(message), remoteEndPoint, 0, ec);
        }
    } catch (const std::exception& e) {
        LOG(ERROR) << format("error: {}", e.what());
    }

    closeLog();
    return 0;
}