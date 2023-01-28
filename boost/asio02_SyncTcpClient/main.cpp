#include <boost/asio.hpp>
#include <common/common.hpp>

using namespace fmt;
using namespace common;
using namespace boost::asio;

int main(int argc, const char* argv[]) {
    initLog(argc, argv);

    try {
        const std::string kHost{"127.0.0.1"};
        io_context io;
        // method 01
        // ip::tcp::resolver resolver(io);
        // ip::tcp::resolver::results_type endPoints = resolver.resolve("", "daytime");
        // ip::tcp::socket socket(io);
        // connect(socket, endPoints);
        // method 02
        ip::tcp::endpoint endPoint(ip::address::from_string(kHost), 1234);
        ip::tcp::socket socket(io);
        socket.connect(endPoint);

        while (true) {
            std::array<char, 128> buf;
            boost::system::error_code ec;
            auto len = socket.read_some(buffer(buf), ec);
            if (ec == error::eof) {
                break;
            } else if (ec) {
                LOG(ERROR) << format("error: {}", ec.message());
                throw boost::system::error_code(ec);
            }
            LOG(INFO) << std::string(buf.data(), len);
        }
    } catch (const std::exception& e) {
        LOG(ERROR) << format("error: {}", e.what());
    }

    closeLog();
    return 0;
}