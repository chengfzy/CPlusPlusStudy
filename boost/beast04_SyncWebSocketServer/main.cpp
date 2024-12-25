#include <fmt/ostream.h>
#include <boost/asio.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>
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

void doSession(tcp::socket&& socket) {
    try {
        // construct the stream by moving the socket
        websocket::stream<tcp::socket> ws{move(socket)};

        // set a decorator to change the server of the handshake
        ws.set_option(websocket::stream_base::decorator([](websocket::request_type& res) {
            res.set(http::field::server, string(BOOST_BEAST_VERSION_STRING) + " websocket-server-sync");
        }));

        // accept the websocket handshake
        ws.accept();

        while (true) {
            // this buffer will host the incoming message
            beast::flat_buffer buffer;
            // read a message into our buffer
            ws.read(buffer);
            LOG(INFO) << format("receive text: {}", fmt::streamed(beast::make_printable(buffer.data())));

            // echo the message back
            ws.text(ws.got_text());
            auto replyText =
                format("##{}##", string(asio::buffers_begin(buffer.data()), asio::buffers_end(buffer.data())));
            ws.write(asio::buffer(replyText));
        }
    } catch (const beast::system_error& ec) {
        if (ec.code() != websocket::error::closed) {
            LOG(ERROR) << format("error: {}", ec.what());
        } else {
            LOG(ERROR) << format("client close, {}", ec.what());
        }
    } catch (const exception& e) {
        LOG(ERROR) << format("error: {}", e.what());
    }
}

int main(int argc, const char* argv[]) {
    initLog(argc, argv);

    try {
        // input parameters
        string host{"127.0.0.1"};
        int port{8800};

        asio::io_context io;

        // the acceptor receivers incoming connections
        tcp::acceptor acceptor(io, tcp::endpoint(asio::ip::make_address(host), port));
        LOG(INFO) << format("server listen address: {}:{}", host, port);

        while (true) {
            // this will receive the new connections
            tcp::socket socket{io};

            // block until we get a connection
            acceptor.accept(socket);
            LOG(INFO) << format("new session coming, address: {}", fmt::streamed(socket.remote_endpoint()));

            thread t(&doSession, move(socket));
            t.detach();
        }
    } catch (const exception& e) {
        LOG(ERROR) << format("error: {}", e.what());
    }

    closeLog();
    return 0;
}