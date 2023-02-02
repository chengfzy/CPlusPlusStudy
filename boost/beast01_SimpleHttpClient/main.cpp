#include <boost/asio.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
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

int main(int argc, const char* argv[]) {
    initLog(argc, argv);

    try {
        // input
        string host{"127.0.0.1"};
        string port{"8080"};
        string target = "/?foo=bar";
        int version = 11;  // 10, 11, HTTP version, 1.0 or 1.1

        asio::io_context io;
        tcp::resolver resolver(io);
        beast::tcp_stream stream(io);

        // look up the domain name
        auto results = resolver.resolve(host, port);
        // make the connection on the IP address we get from a lookup
        stream.connect(results);
        // set up an HTTP GET request message
        http::request<http::string_body> req{http::verb::get, target, version};
        req.set(http::field::host, host);
        req.set(http::field::user_agent, BOOST_BEAST_VERSION_STRING);
        // send the HTTP request to the remote host
        http::write(stream, req);
        // the buffer is used fro reading and must be persisted
        beast::flat_buffer buffer;
        // declare a container to hold the response
        http::response<http::dynamic_body> res;
        // receive the HTTP response
        http::read(stream, buffer, res);
        // write the message to standard out
        LOG(INFO) << res;
        // gracefully close the socket
        beast::error_code ec;
        stream.socket().shutdown(tcp::socket::shutdown_both, ec);

        // not connected happens sometimes, so don't bother reporting it
        if (ec && ec != beast::errc::not_connected) {
            throw beast::system_error(ec);
        }
    } catch (const exception& e) {
        LOG(ERROR) << format("error: {}", e.what());
    }

    closeLog();
    return 0;
}