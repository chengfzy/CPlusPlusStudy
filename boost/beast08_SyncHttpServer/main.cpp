#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>
#include <boost/filesystem.hpp>
#include <common/common.hpp>
#include <thread>

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
 * @brief This function produces an HTTP response for the given request, the type of the response object depends on the
 * contents of the request, so the interface requires the caller to pass a generic lambda for receiving the response
 */
template <class Body, class Allocator, class Sender>
void handleRequest(beast::string_view docRoot, http::request<Body, http::basic_fields<Allocator>>&& req,
                   Sender&& sender) {
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
        return sender(badRequest("unknown HTTP method"));
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
        return sender(notFound(req.target()));
    }
    // handle an unknown error
    if (ec) {
        return sender(serverError(ec.message()));
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
        return sender(std::move(res));
    } else {
        // respond to GET request
        http::response<http::file_body> res(piecewise_construct, make_tuple(move(body)),
                                            make_tuple(http::status::ok, req.version()));
        res.set(http::field::server, BOOST_BEAST_VERSION_STRING);
        res.set(http::field::content_type, "application/text");
        res.content_length(size);
        res.keep_alive(req.keep_alive());
        return sender(std::move(res));
    }
}

template <class Stream>
struct Sender {
  public:
    explicit Sender(Stream& stream, bool& close, beast::error_code& ec) : stream_(stream), close_(close), ec_(ec) {}

  public:
    template <bool isRequest, class Body, class Fields>
    void operator()(http::message<isRequest, Body, Fields>&& msg) const {
        // determine if we should close the connection after
        close_ = msg.need_eof();

        // we need the serializer here because the serializer requires a non-const file body, and the message oriented
        // version of http::write only works with const message
        http::serializer<isRequest, Body, Fields> sr(msg);
        http::write(stream_, sr, ec_);
    }

  private:
    Stream& stream_;
    bool& close_;
    beast::error_code& ec_;
};

void doSession(tcp::socket&& socket, const string& docRoot) {
    beast::error_code ec;
    bool close{false};
    beast::flat_buffer buffer;

    Sender<tcp::socket> sender(socket, close, ec);

    while (true) {
        // read a request
        http::request<http::string_body> req;
        http::read(socket, buffer, req, ec);
        if (ec == http::error::end_of_stream) {
            break;
        }
        if (ec) {
            return error(ec, "read");
        }

        // parser request
        LOG(INFO) << format("request target: {}", req.target());

        // send the response
        handleRequest(docRoot, move(req), sender);
        if (ec) {
            return error(ec, "write");
        }
        if (close) {
            // this means we should close the connection, usually because the response indicated the "Connection: close"
            // semantic
            break;
        }

        // send a TCP shutdown
        socket.shutdown(tcp::socket::shutdown_send, ec);
    }
}

int main(int argc, const char* argv[]) {
    initLog(argc, argv);

    try {
        // input
        string host{"127.0.0.1"};
        int port{8080};
        string docRoot{"."};

        asio::io_context io(1);

        // the acceptor receives incoming connections
        tcp::acceptor acceptor(io, tcp::endpoint(asio::ip::make_address(host), port));
        while (true) {
            // this will receive the new connection
            tcp::socket socket(io);

            // block until we get a new connection
            acceptor.accept(socket);

            // launch the session, transferring ownership of the socket
            thread(&doSession, move(socket), docRoot).detach();
        }
    } catch (const exception& e) {
        LOG(ERROR) << format("error: {}", e.what());
    }

    closeLog();
    return 0;
}