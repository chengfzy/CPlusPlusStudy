#include <fmt/ostream.h>
#include <boost/asio/strand.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <common/common.hpp>

using namespace std;
using namespace fmt;
using namespace common;
namespace beast = boost::beast;
namespace http = boost::beast::http;
namespace websocket = beast::websocket;
namespace asio = boost::asio;
using tcp = boost::asio::ip::tcp;

/**
 * @brief Send a WebSocket message and print the response
 *
 */
class Session : public enable_shared_from_this<Session> {
  public:
    explicit Session(asio::io_context& io) : resolver_(asio::make_strand(io)), ws_(asio::make_strand(io)) {}

  public:
    void run(const string& host, const string& port) {
        // save host
        host_ = host;

        // look up the domain name
        resolver_.async_resolve(host, port, beast::bind_front_handler(&Session::onResolve, shared_from_this()));
    }

  private:
    void onResolve(const beast::error_code& ec, const tcp::resolver::results_type& results) {
        if (ec) {
            return error(ec, "connect");
        }

        // set the timeout for the operation
        beast::get_lowest_layer(ws_).expires_after(30s);

        // make the connection on the IP address we get from a lookup
        beast::get_lowest_layer(ws_).async_connect(results,
                                                   beast::bind_front_handler(&Session::onConnect, shared_from_this()));
    }

    void onConnect(const beast::error_code& ec, const tcp::resolver::results_type::endpoint_type& ep) {
        if (ec) {
            return error(ec, "connect");
        }

        // turn off the timeout on the tcp stream, because the WebSocket stream has its own timeout system
        beast::get_lowest_layer(ws_).expires_never();

        // set suggested timeout settings fro the WebSocket
        ws_.set_option(websocket::stream_base::timeout::suggested(beast::role_type::client));

        // set a decorator to change the User-Agent of the handshake
        ws_.set_option(websocket::stream_base::decorator([](websocket::request_type& req) {
            req.set(http::field::user_agent, string(BOOST_BEAST_VERSION_STRING) + " websocket-client-async");
        }));

        // update the host_ string, this will provide the value of the host HTTP header during WebSocket handshake. see
        // https://tools.ietf.org/html/rfc7230#section-5.4
        host_ += ':' + to_string(ep.port());
        LOG(INFO) << format("host: {}", host_);

        // perform the WebSocket handshake
        ws_.async_handshake(host_, "/", beast::bind_front_handler(&Session::onHandshake, shared_from_this()));
    }

    void onHandshake(const beast::error_code& ec) {
        if (ec) {
            return error(ec, "handshake");
        }

        // send the message
        static int index;
        auto text = format("Hello, WebSocket, n = {}", index++);
        ws_.async_write(asio::buffer(text), beast::bind_front_handler(&Session::onWrite, shared_from_this()));
    }

    void onWrite(const beast::error_code& ec, const std::size_t& bytesTransferred) {
        boost::ignore_unused(bytesTransferred);

        if (ec) {
            return error(ec, "write");
        }

        // read a message into our buffer
        ws_.async_read(buffer_, beast::bind_front_handler(&Session::onRead, shared_from_this()));
    }

    void onRead(const beast::error_code& ec, const std::size_t& bytesTransferred) {
        boost::ignore_unused(bytesTransferred);

        if (ec) {
            return error(ec, "read");
        }

        // close the WebSocket connection
        ws_.async_close(websocket::close_code::normal,
                        beast::bind_front_handler(&Session::onClose, shared_from_this()));
    }

    void onClose(const beast::error_code& ec) {
        if (ec) {
            return error(ec, "close");
        }

        LOG(INFO) << format("receive text: {}", fmt::streamed(beast::make_printable(buffer_.data())));
    }

    void error(const beast::error_code& ec, const string& type) {
        LOG(ERROR) << format("{} error: {}", type, ec.message());
    }

  private:
    tcp::resolver resolver_;
    websocket::stream<beast::tcp_stream> ws_;
    beast::flat_buffer buffer_;
    string host_;
};

int main(int argc, const char* argv[]) {
    initLog(argc, argv);

    // input parameters
    string host{"127.0.0.1"};
    string port{"8800"};

    // launch the asynchronous operation
    asio::io_context io;
    make_shared<Session>(io)->run(host, port);

    io.run();

    closeLog();
    return 0;
}