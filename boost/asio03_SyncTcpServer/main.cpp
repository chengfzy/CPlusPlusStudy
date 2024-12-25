#include <fmt/ostream.h>
#include <boost/asio.hpp>
#include <common/common.hpp>

using namespace fmt;
using namespace common;
using namespace boost::asio;

int main(int argc, const char* argv[]) {
    initLog(argc, argv);

    // function to obtain a string
    auto dayTimeStrFunc = []() {
        static int count{0};
        return format("count = {}", count++);
    };

    try {
        io_context io;
        ip::tcp::acceptor acceptor(io, ip::tcp::endpoint(ip::tcp::v4(), 1234));  // TCP port 1234 for IPv4
        LOG(INFO) << format("start TCP server on {}", fmt::streamed(acceptor.local_endpoint()));
        while (true) {
            ip::tcp::socket socket(io);
            acceptor.accept(socket);
            auto message = dayTimeStrFunc();
            boost::system::error_code ec;
            write(socket, boost::asio::buffer(message), ec);
        }
    } catch (const std::exception& e) {
        LOG(ERROR) << format("error: {}", e.what());
    }

    closeLog();
    return 0;
}