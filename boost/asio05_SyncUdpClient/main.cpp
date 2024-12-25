#include <fmt/ostream.h>
#include <boost/asio.hpp>
#include <common/common.hpp>

using namespace fmt;
using namespace common;
using namespace boost::asio;

int main(int argc, const char* argv[]) {
    initLog(argc, argv);

    try {
        const std::string kHost{"127.0.0.1"};
        io_context io;
        ip::udp::resolver resolver(io);
        ip::udp::endpoint receiveEndPoint(ip::address::from_string(kHost), 1234);

        ip::udp::socket socket(io);
        socket.open(ip::udp::v4());

        LOG(INFO) << format("initiate contact with endpoint {}", fmt::streamed(receiveEndPoint));
        std::array<char, 1> sendBuf = {{0}};
        socket.send_to(buffer(sendBuf), receiveEndPoint);
        LOG(INFO) << format("receive endpoint: {}", fmt::streamed(receiveEndPoint));

        std::array<char, 128> recvBuf;
        ip::udp::endpoint sendEndPoint;
        auto len = socket.receive_from(buffer(recvBuf), sendEndPoint);
        LOG(INFO) << format("receive data from {}: {}", fmt::streamed(sendEndPoint), std::string(recvBuf.data(), len));
    } catch (const std::exception& e) {
        LOG(ERROR) << format("error: {}", e.what());
    }

    closeLog();
    return 0;
}