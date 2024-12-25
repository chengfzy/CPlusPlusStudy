#include <fmt/ostream.h>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <common/common.hpp>

using namespace fmt;
using namespace common;
using namespace boost::asio;

class UdpServer {
  public:
    explicit UdpServer(io_context& io) : socket_(io, ip::udp::endpoint(ip::udp::v4(), 1234)) { startReceive(); }

  public:
    void startReceive() {
        socket_.async_receive_from(
            buffer(recvBuffer_), remoteEndPoint_,
            boost::bind(&UdpServer::handleReceive, this, placeholders::error, placeholders::bytes_transferred));
    }

    void handleReceive(const boost::system::error_code& ec, size_t bytesTransferred) {
        if (!ec) {
            static int count{0};
            auto message = format("count = {}", count++);
            LOG(INFO) << format("send message: \"{}\"", message);

            LOG(INFO) << format("remote endpoint: {}", fmt::streamed(remoteEndPoint_));
            socket_.async_send_to(buffer(message), remoteEndPoint_, std::bind(&UdpServer::handleSend, this));
            startReceive();
        }
    }

    void handleSend() {}

  private:
    ip::udp::socket socket_;
    ip::udp::endpoint remoteEndPoint_;
    std::array<char, 1> recvBuffer_;
};

int main(int argc, const char* argv[]) {
    initLog(argc, argv);

    try {
        io_context io;
        UdpServer server(io);
        io.run();
    } catch (const std::exception& e) {
        LOG(ERROR) << format("error: {}", e.what());
    }

    closeLog();
    return 0;
}