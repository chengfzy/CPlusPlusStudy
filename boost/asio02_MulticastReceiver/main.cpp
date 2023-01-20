#include <boost/asio.hpp>
#include <chrono>
#include <iostream>
#include <sstream>
#include <string>

using namespace std;
using namespace boost::asio;

constexpr int kMulticastPort{30001};

/**
 * @brief Receiver
 */
class Receiver {
  public:
    Receiver(boost::asio::io_context& ioContext, const ip::address& listenAddress, const ip::address& multicastAddress)
        : socket_(ioContext) {
        // create the socket so that multiple may be bound to the same address
        ip::udp::endpoint listenEndpoint(listenAddress, kMulticastPort);
        socket_.open(listenEndpoint.protocol());
        socket_.set_option(ip::udp::socket::reuse_address(true));
        socket_.bind(listenEndpoint);

        // join the multicast group
        socket_.set_option(ip::multicast::join_group(multicastAddress));

        doReceive();
    }

  private:
    void doReceive() {
        socket_.async_receive_from(buffer(data_), senderEndpoint_,
                                   [this](boost::system::error_code ec, std::size_t length) {
                                       if (!ec) {
                                           cout.write(data_.data(), length);
                                           cout << endl;
                                           doReceive();
                                       }
                                   });
    }

  private:
    ip::udp::endpoint senderEndpoint_;
    ip::udp::socket socket_;
    std::array<char, 1024> data_;
};

int main(int argc, char* argv[]) {
    cout << "Usage: Receiver <listen_address> <multicast_address>" << endl;
    cout << "\tFor IPv4: try: Receiver 0.0.0.0 239.255.0.1" << endl;
    cout << "\tFor IPv6: try: Receiver 0::0 ff31::8000:1234" << endl;

    io_context ioContext;
    Receiver receiver(ioContext, ip::make_address("0.0.0.0"), ip::make_address("239.255.0.1"));
    ioContext.run();

    return 0;
}