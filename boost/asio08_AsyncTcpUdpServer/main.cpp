#include <fmt/ostream.h>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <common/common.hpp>

using namespace fmt;
using namespace common;
using namespace boost::asio;

class TcpConnection : public std::enable_shared_from_this<TcpConnection> {
  public:
    static std::shared_ptr<TcpConnection> create(io_context& io) {
        return std::shared_ptr<TcpConnection>(new TcpConnection(io));
    }

    ip::tcp::socket& socket() { return socket_; }

    void start() {
        static int count{0};
        message_ = format("count = {}", count++);
        LOG(INFO) << format("TCP send message: \"{}\"", message_);

        // method 01
        // async_write(socket_, buffer(message_),
        //             boost::bind(&TcpConnection::handleWrite01, shared_from_this(), placeholders::error,
        //                         placeholders::bytes_transferred));
        // method 02
        async_write(socket_, buffer(message_), boost::bind(&TcpConnection::handleWrite02, shared_from_this()));
    }

    void handleWrite01(const boost::system::error_code& ec, size_t bytesTransferred) {}
    void handleWrite02() {}

  private:
    explicit TcpConnection(io_context& io) : socket_(io) {}

  private:
    ip::tcp::socket socket_;
    std::string message_;
};

class TcpServer {
  public:
    explicit TcpServer(io_context& io) : io_(io), acceptor_(io, ip::tcp::endpoint(ip::tcp::v4(), 1234)) {
        startAccept();
    }

  public:
    // create a socket and initiates an asynchronous accept operation to wait for a new connection
    void startAccept() {
        auto newConnection = TcpConnection::create(io_);
        acceptor_.async_accept(newConnection->socket(),
                               boost::bind(&TcpServer::handleAccept, this, newConnection, placeholders::error));
    }

    // services the client request, and then call startAccept() to initiate the next accept operation
    void handleAccept(std::shared_ptr<TcpConnection>& newConnection, const boost::system::error_code& error) {
        if (!error) {
            newConnection->start();
        }
        startAccept();
    }

  private:
    io_context& io_;
    ip::tcp::acceptor acceptor_;
};

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
            LOG(INFO) << format("UDP send message: \"{}\"", message);

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
        TcpServer server1(io);
        UdpServer server2(io);

        io.run();
    } catch (const std::exception& e) {
        LOG(ERROR) << format("error: {}", e.what());
    }

    closeLog();
    return 0;
}