#include <boost/asio.hpp>
#include <chrono>
#include <iostream>
#include <sstream>
#include <string>

using namespace std;
using namespace boost::asio;

constexpr int kMulticastPort{30001};
constexpr int kMaxMessageCount{100};

/**
 * @brief Sender
 */
class Sender {
  public:
    Sender(boost::asio::io_context& ioContext, const ip::address& multicastAddress)
        : endpoint_(multicastAddress, kMulticastPort),
          socket_(ioContext, endpoint_.protocol()),
          timer_(ioContext),
          messageCount_(0) {
        doSend();
    }

  private:
    void doSend() {
        std::ostringstream os;
        os << "Message " << messageCount_++;
        message_ = os.str();

        socket_.async_send_to(buffer(message_), endpoint_, [this](boost::system::error_code ec, std::size_t length) {
            if (!ec && messageCount_ < kMaxMessageCount) {
            }
            doTimeout();
        });
    }

    void doTimeout() {
        timer_.expires_after(std::chrono::seconds(1));
        timer_.async_wait([this](boost::system::error_code ec) {
            if (!ec) {
                doSend();
            }
        });
    }

  private:
    ip::udp::endpoint endpoint_;
    ip::udp::socket socket_;
    steady_timer timer_;
    int messageCount_;
    std::string message_;
};

int main(int argc, char* argv[]) {
    cout << "Usage: Sender <multicast_address>" << endl;
    cout << "\tFor IPv4: try: Sender 239.255.0.1" << endl;
    cout << "\tFor IPv6: try: Sender ff31::8000:1234" << endl;

    io_service ioService;
    Sender sender(ioService, ip::make_address("239.255.0.1"));
    ioService.run();

    return 0;
}