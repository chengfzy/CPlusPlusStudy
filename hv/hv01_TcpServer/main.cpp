#include <fmt/format.h>
#include <hv/TcpServer.h>
#include <boost/lexical_cast.hpp>
#include <common/common.hpp>
#include <iostream>

using namespace std;
using namespace fmt;
using namespace common;
using namespace hv;

int main(int argc, const char* argv[]) {
    initLog(argc, argv);

    // disable log
    hlog_disable();

    constexpr int kPort{60010};
    TcpServer server;
    auto listenFd = server.createsocket(kPort);
    CHECK(listenFd >= 0) << format("create TCP server failed, fd({}) < 0", listenFd);
    LOG(INFO) << format("server listen on port {}, listen fd = {}", kPort, listenFd);

    server.onConnection = [](const SocketChannelPtr& channel) {
        auto peerAddr = channel->peeraddr();
        if (channel->isConnected()) {
            LOG(INFO) << format("{} connected, connect fd = {}", peerAddr, channel->fd());
        } else {
            LOG(INFO) << format("{} disconnected, connect fd = {}", peerAddr, channel->fd());
        }
    };
    server.onMessage = [](const SocketChannelPtr& channel, Buffer* buf) {
        static int m{0};
        LOG(INFO) << format("receive (len={}): {}", buf->size(), static_cast<char*>(buf->data()));
        auto n = boost::lexical_cast<int>(static_cast<char*>(buf->data()) + 4, buf->size() - 4);
        if (n % 3 == 0) {
            channel->write(format("m = {}", m));
            ++m;
        }
    };
    server.onWriteComplete = [](const SocketChannelPtr& channel, Buffer* buf) {
        LOG(INFO) << format("finish write(len={}): {}", buf->size(), static_cast<char*>(buf->data()));
    };
    server.setThreadNum(4);
    server.start();

    while (true) {
        this_thread::sleep_for(1s);
    }

    closeLog();
    return 0;
}