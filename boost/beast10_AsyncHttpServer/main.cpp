#include <fmt/ostream.h>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/strand.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>
#include <boost/filesystem.hpp>
#include <common/common.hpp>
#include <thread>

#include <boost/asio.hpp>
#include <boost/beast.hpp>

using namespace std;
using namespace fmt;
using namespace common;
namespace beast = boost::beast;
namespace http = boost::beast::http;
namespace asio = boost::asio;
namespace fs = boost::filesystem;
using tcp = boost::asio::ip::tcp;

void error(const beast::error_code& ec, const string& type) {
    LOG(ERROR) << format("{} error: {}", type, ec.message());
}

/**
 * @brief This function produces an HTTP response for the given request. The concrete type of the response message(which
 * depends on the request), is type-erased in message_generator
 */
template <class Body, class Allocator>
http::message_generator handleRequest(beast::string_view docRoot,
                                      http::request<Body, http::basic_fields<Allocator>>&& req) {
    // return a bad request response
    auto badRequest = [&req](const beast::string_view& why) {
        http::response<http::string_body> res(http::status::bad_request, req.version());
        res.set(http::field::server, BOOST_BEAST_VERSION_STRING);
        res.set(http::field::content_type, "text/html");
        res.keep_alive(req.keep_alive());
        res.body() = string(why);
        res.prepare_payload();
        return res;
    };

    // return a not found response
    auto notFound = [&req](const beast::string_view& target) {
        http::response<http::string_body> res(http::status::not_found, req.version());
        res.set(http::field::server, BOOST_BEAST_VERSION_STRING);
        res.set(http::field::content_type, "text/html");
        res.keep_alive(req.keep_alive());
        res.body() = format("the resource {} was not found", target);
        res.prepare_payload();
        return res;
    };

    // return a server error response
    auto serverError = [&req](const beast::string_view& what) {
        http::response<http::string_body> res(http::status::internal_server_error, req.version());
        res.set(http::field::server, BOOST_BEAST_VERSION_STRING);
        res.set(http::field::content_type, "text/html");
        res.keep_alive(req.keep_alive());
        res.body() = format("an error occurred: ", what);
        res.prepare_payload();
        return res;
    };

    // make sure we handle the method
    if (req.method() != http::verb::get && req.method() != http::verb::head) {
        LOG(ERROR) << "unknown HTTP method";
        return badRequest("unknown HTTP method");
    }

    // build the path to the requested file
    fs::path path = fs::path(string(docRoot)) / fs::path(string(req.target()));
    if (req.target().back() == '/') {
        path.replace_extension("index.html");
    }

    // attempt to open the file
    beast::error_code ec;
    http::file_body::value_type body;
    body.open(path.c_str(), beast::file_mode::scan, ec);
    // handle the case where the file doesn't exist
    if (ec == beast::errc::no_such_file_or_directory) {
        return notFound(req.target());
    }
    // handle an unknown error
    if (ec) {
        return serverError(ec.message());
    }

    // cache the size since we need it after the move
    auto size = body.size();
    if (req.method() == http::verb::head) {
        // respond to HEAD request
        http::response<http::empty_body> res(http::status::ok, req.version());
        res.set(http::field::server, BOOST_BEAST_VERSION_STRING);
        res.set(http::field::content_type, "application/text");
        res.content_length(size);
        res.keep_alive(req.keep_alive());
        return res;
    } else {
        // respond to GET request
        http::response<http::file_body> res(piecewise_construct, make_tuple(move(body)),
                                            make_tuple(http::status::ok, req.version()));
        res.set(http::field::server, BOOST_BEAST_VERSION_STRING);
        res.set(http::field::content_type, "application/text");
        res.content_length(size);
        res.keep_alive(req.keep_alive());
        return res;
    }
}

/**
 * @brief Handles a HTTP server connection
 *
 */
class Session : public enable_shared_from_this<Session> {
  public:
    // take ownership of the socket
    explicit Session(tcp::socket&& socket, const string& docRoot) : stream_(move(socket)), docRoot_(docRoot) {
        LOG(INFO) << format("new session coming, address: {}", stream_.socket().remote_endpoint());
    }

  public:
    // start the asynchronous operation
    void run() {
        // we need to be executing within a strand to perform async operations on the IO objects in the session,
        // although not strictly necessary for single-threaded contexts, this example code is written to be thread-safe
        // by default.
        asio::dispatch(stream_.get_executor(), beast::bind_front_handler(&Session::doRead, shared_from_this()));
    }

  private:
    void doRead() {
        // make the request empty before reading, otherwise the operation behavior is undefined
        req_ = {};

        // set the time out
        stream_.expires_after(30s);

        // read a request
        http::async_read(stream_, buffer_, req_, beast::bind_front_handler(&Session::onRead, shared_from_this()));
    }

    void onRead(const beast::error_code& ec, const std::size_t& bytesTransferred) {
        boost::ignore_unused(bytesTransferred);

        // the connection is closed
        if (ec == http::error::end_of_stream) {
            LOG(WARNING) << "session closed";
            return doClose();
        }

        if (ec) {
            return error(ec, "read");
        }

        LOG(INFO) << format("request target: {}", req_.target());

        // send the response
        sendResponse(handleRequest(docRoot_, move(req_)));
    }

    void sendResponse(http::message_generator&& msg) {
        // write the response
        auto keepAlive = msg.keep_alive();
        beast::async_write(stream_, move(msg),
                           beast::bind_front_handler(&Session::onWrite, shared_from_this(), keepAlive));
    }

    void onWrite(bool keepAlive, const beast::error_code& ec, const std::size_t& bytesTransferred) {
        boost::ignore_unused(bytesTransferred);

        if (ec) {
            return error(ec, "write");
        }

        if (!keepAlive) {
            // this means we should close the connection, usually because the response indicated the "Connection: close"
            // semantic
            return doClose();
        }

        // read another request
        doRead();
    }

    void doClose() {
        // send a TCP shutdown
        beast::error_code ec;
        stream_.socket().shutdown(tcp::socket::shutdown_send, ec);
    }

  private:
    beast::tcp_stream stream_;
    beast::flat_buffer buffer_;
    const string docRoot_;
    http::request<http::string_body> req_;
};

/**
 * @brief Accepts incoming connections and launches the sessions
 *
 */
class Listener : public enable_shared_from_this<Listener> {
  public:
    Listener(asio::io_context& io, const tcp::endpoint& endpoint, const string& docRoot)
        : io_(io), acceptor_(asio::make_strand(io)), docRoot_(docRoot) {
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
            make_shared<Session>(move(socket), docRoot_)->run();
        }

        // accept another connection
        doAccept();
    }

  private:
    asio::io_context& io_;
    tcp::acceptor acceptor_;
    string docRoot_;
};

int main(int argc, const char* argv[]) {
    initLog(argc, argv);

    try {
        // input
        string host{"127.0.0.1"};
        int port{8080};
        string docRoot{"."};
        constexpr int kThreadNum{4};

        asio::io_context io(kThreadNum);

        // create and launch a listening port
        make_shared<Listener>(io, tcp::endpoint(asio::ip::make_address(host), port), docRoot)->run();

        // run the IO service on the requested number of threads
        vector<thread> threads;
        for (size_t i = 0; i < kThreadNum; ++i) {
            threads.emplace_back([&io] { io.run(); });
        }
        io.run();
    } catch (const exception& e) {
        LOG(ERROR) << format("error: {}", e.what());
    }

    closeLog();
    return 0;
}