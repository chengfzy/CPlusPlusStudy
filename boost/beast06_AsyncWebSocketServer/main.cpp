#include <fmt/ostream.h>
#include <boost/asio/buffers_iterator.hpp>
#include <boost/asio/dispatch.hpp>
#include <boost/asio/strand.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <common/common.hpp>
#include <thread>

using namespace std;
using namespace fmt;
using namespace common;
namespace beast = boost::beast;
namespace http = boost::beast::http;
namespace websocket = beast::websocket;
namespace asio = boost::asio;
using tcp = boost::asio::ip::tcp;

void error(const beast::error_code& ec, const string& type) {
    LOG(ERROR) << format("{} error: {}", type, ec.message());
}

/**
 * @brief Echoes back all received WebSocket messages
 *
 */
class Session : public enable_shared_from_this<Session> {
  public:
    // take ownership of the socket
    explicit Session(tcp::socket&& socket) : ws_(move(socket)) {
        LOG(INFO) << format("new session coming, address: {}",
                            fmt::streamed(ws_.next_layer().socket().remote_endpoint()));
    }

  public:
    // get on the correct executor
    void run() {
        // we need to be executing within a strand to perform async operations on the IO objects in the session,
        // although not strictly necessary for single-threaded contexts, this example code is written to be thread-safe
        // by default.
        asio::dispatch(ws_.get_executor(), beast::bind_front_handler(&Session::onRun, shared_from_this()));
    }

  private:
    // start the asynchronous operation
    void onRun() {
        // set suggested timeout settings for the WebSocket
        ws_.set_option(websocket::stream_base::timeout::suggested(beast::role_type::server));

        // set a decorator to change the server for the handshake
        ws_.set_option(websocket::stream_base::decorator([](websocket::request_type& res) {
            res.set(http::field::server, string(BOOST_BEAST_VERSION_STRING) + " websocket-server-async");
        }));

        // accept the websocket handshake
        ws_.async_accept(beast::bind_front_handler(&Session::onAccept, shared_from_this()));
    }

    void onAccept(const beast::error_code& ec) {
        if (ec) {
            return error(ec, "accept");
        }

        // read a message
        doRead();
    }

    void doRead() {
        // read a message into our buffer
        ws_.async_read(buffer_, beast::bind_front_handler(&Session::onRead, shared_from_this()));
    }

    void onRead(const beast::error_code& ec, const std::size_t& bytesTransferred) {
        boost::ignore_unused(bytesTransferred);

        // when session was closed
        if (ec == websocket::error::closed) {
            LOG(WARNING) << "session closed";
            return;
        }

        if (ec) {
            return error(ec, "read");
        }

        LOG(INFO) << format("receive text: {}", fmt::streamed(beast::make_printable(buffer_.data())));

        // echo the message back
        ws_.text(ws_.got_text());
        auto replyText =
            format("##{}##", string(asio::buffers_begin(buffer_.data()), asio::buffers_end(buffer_.data())));
        // ws_.async_write(buffer_.data(), beast::bind_front_handler(&Session::onWrite, shared_from_this()));
        ws_.async_write(asio::buffer(replyText), beast::bind_front_handler(&Session::onWrite, shared_from_this()));
    }

    void onWrite(const beast::error_code& ec, const std::size_t& bytesTransferred) {
        boost::ignore_unused(bytesTransferred);

        if (ec) {
            return error(ec, "write");
        }

        // clear the buffer
        buffer_.consume(buffer_.size());

        // do another read
        doRead();
    }

  private:
    websocket::stream<beast::tcp_stream> ws_;
    beast::flat_buffer buffer_;
};

/**
 * @brief Accept incoming connections and launches the sessions
 *
 */
class Listener : public enable_shared_from_this<Listener> {
  public:
    Listener(asio::io_context& io, const tcp::endpoint& endpoint) : io_(io), acceptor_(io) {
        LOG(INFO) << format("server address: {}", fmt::streamed(endpoint));
        beast::error_code ec;

        // open the acceptor
        acceptor_.open(endpoint.protocol(), ec);
        if (ec) {
            error(ec, "open");
            return;
        }

        // allow address reuse
        acceptor_.set_option(asio::socket_base::reuse_address(true), ec);
        if (ec) {
            error(ec, "set_option");
            return;
        }

        // bind to the server address
        acceptor_.bind(endpoint, ec);
        if (ec) {
            error(ec, "bind");
            return;
        }

        // start listening for connections
        acceptor_.listen(asio::socket_base::max_listen_connections, ec);
        if (ec) {
            error(ec, "listen ");
            return;
        }
    }

    // start accepting incoming connections
    void run() { doAccept(); }

  private:
    void doAccept() {
        // the new connections gets its own strand
        acceptor_.async_accept(asio::make_strand(io_),
                               beast::bind_front_handler(&Listener::onAccept, shared_from_this()));
    }

    void onAccept(const beast::error_code& ec, tcp::socket&& socket) {
        if (ec) {
            error(ec, "accept");
        } else {
            // create the session and run
            make_shared<Session>(move(socket))->run();
        }

        // accept another connection
        doAccept();
    }

  private:
    asio::io_context& io_;
    tcp::acceptor acceptor_;
};

int main(int argc, const char* argv[]) {
    initLog(argc, argv);

    try {
        // input parameters
        string host{"127.0.0.1"};
        int port{8800};
        const int kThreadNum{4};  // thread number

        asio::io_context io(kThreadNum);

        // create and launch a listening port
        make_shared<Listener>(io, tcp::endpoint(asio::ip::make_address(host), port))->run();

        // run the io service on the requested number of threads
        vector<thread> threads;
        threads.reserve(kThreadNum - 1);
        for (size_t i = 0; i < kThreadNum; ++i) {
            threads.emplace_back(thread([&io] { io.run(); }));
        }
        io.run();
    } catch (const exception& e) {
        LOG(ERROR) << format("error: {}", e.what());
    }

    closeLog();
    return 0;
}