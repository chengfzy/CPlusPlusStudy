#include <fmt/ostream.h>
#include <boost/asio.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/strand.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>
#include <common/common.hpp>

using namespace std;
using namespace fmt;
using namespace common;
namespace beast = boost::beast;
namespace http = boost::beast::http;
namespace asio = boost::asio;
using tcp = boost::asio::ip::tcp;

/**
 * @brief Performs a HTTP GEt and prints the response
 *
 */
class Session : public enable_shared_from_this<Session> {
  public:
    explicit Session(asio::io_context& io) : resolver_(asio::make_strand(io)), stream_(asio::make_strand(io)) {}

  public:
    void run(const string& host, int port, const string& target, int version) {
        // set up and HTTP GET request message
        req_.version(version);
        req_.method(http::verb::get);
        req_.target(target);
        req_.set(http::field::host, host);
        req_.set(http::field::user_agent, BOOST_BEAST_VERSION_STRING);

        // look up the domain name
        resolver_.async_resolve(host, to_string(port),
                                beast::bind_front_handler(&Session::onResolve, shared_from_this()));
    }

  private:
    void onResolve(const beast::error_code& ec, const tcp::resolver::results_type& results) {
        if (ec) {
            return error(ec, "connect");
        }

        // set the timeout for the operation
        stream_.expires_after(30s);

        // make the connection on the IP address we get from a lookup
        stream_.async_connect(results, beast::bind_front_handler(&Session::onConnect, shared_from_this()));
    }

    void onConnect(const beast::error_code& ec, const tcp::resolver::results_type::endpoint_type& ep) {
        if (ec) {
            return error(ec, "connect");
        }

        // Set a timeout on the operation
        stream_.expires_after(std::chrono::seconds(30));

        // send the HTTP request to the remote host
        http::async_write(stream_, req_, beast::bind_front_handler(&Session::onWrite, shared_from_this()));
    }

    void onWrite(const beast::error_code& ec, const std::size_t& bytesTransferred) {
        boost::ignore_unused(bytesTransferred);

        if (ec) {
            return error(ec, "write");
        }

        // receive the HTTP response
        http::async_read(stream_, buffer_, res_, beast::bind_front_handler(&Session::onRead, shared_from_this()));
    }

    void onRead(beast::error_code& ec, const std::size_t& bytesTransferred) {
        boost::ignore_unused(bytesTransferred);

        if (ec) {
            return error(ec, "read");
        }

        // print the message
        LOG(INFO) << format("receive response:\n{}", fmt::streamed(res_));

        // gracefully close the socket
        stream_.socket().shutdown(tcp::socket::shutdown_both, ec);

        // not connected happens sometimes so don't bother reporting it
        if (ec == beast::errc::not_connected) {
            return error(ec, "close, not connected");
        } else if (ec) {
            return error(ec, "close");
        }
    }

    void error(const beast::error_code& ec, const string& type) {
        LOG(ERROR) << format("{} error: {}", type, ec.message());
    }

  private:
    tcp::resolver resolver_;
    beast::tcp_stream stream_;
    beast::flat_buffer buffer_;
    http::request<http::empty_body> req_;
    http::response<http::string_body> res_;
};

int main(int argc, const char* argv[]) {
    initLog(argc, argv);

    try {
        // input
        string host{"127.0.0.1"};
        int port{8080};
        string target = "/?foo=bar";
        int version = 11;  // 10, 11, HTTP version, 1.0 or 1.1

        asio::io_context io;

        // launch the asynchronous operation
        make_shared<Session>(io)->run(host, port, target, version);

        io.run();
    } catch (const exception& e) {
        LOG(ERROR) << format("error: {}", e.what());
    }

    closeLog();
    return 0;
}