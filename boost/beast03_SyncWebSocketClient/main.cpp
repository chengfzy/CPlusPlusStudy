#include <fmt/ostream.h>
#include <boost/asio.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>
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

int main(int argc, const char* argv[]) {
    initLog(argc, argv);

    try {
        // input parameters
        string host{"127.0.0.1"};
        string port{"8800"};

        asio::io_context io;
        tcp::resolver resolver(io);
        websocket::stream<tcp::socket> ws{io};

        // look up the domain name
        auto results = resolver.resolve(host, port);
        // make the connection on the IP address we get from a lookup
        auto ep = asio::connect(ws.next_layer(), results);
        // update the host_ string, this will provide the value of the host HTTP header during WebSocket handshake. see
        // https://tools.ietf.org/html/rfc7230#section-5.4
        host += ':' + to_string(ep.port());
        LOG(INFO) << format("host: {}", host);

        // set a decorator to change the User-Agent of the handshake
        ws.set_option(websocket::stream_base::decorator([](websocket::request_type& req) {
            // req.set(http::field::user_agent, format("{} websocket-client-coro", BOOST_BEAST_VERSION_STRING));
            req.set(http::field::user_agent, string(BOOST_BEAST_VERSION_STRING) + " websocket-client-coro");
        }));

        // perform the WebSocket handshake
        ws.handshake(host, "/");

        while (true) {
            static int index;
            auto text = format("Hello, WebSocket, n = {}", index++);
            LOG(INFO) << format("send text: {}", text);
            // send the message
            ws.write(asio::buffer(text));

            // this buffer will host the incoming message
            beast::flat_buffer buffer;
            // read a message into our buffer
            ws.read(buffer);
            LOG(INFO) << format("receive text: {}", fmt::streamed(beast::make_printable(buffer.data())));

            std::this_thread::sleep_for(1s);
        }

        // close the WebSocket connection
        ws.close((websocket::close_code::normal));
    } catch (const exception& e) {
        LOG(ERROR) << format("error: {}", e.what());
    }

    closeLog();
    return 0;
}