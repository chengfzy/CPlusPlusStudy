#include <fmt/format.h>
#include <hv/TcpClient.h>
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
    TcpClient client;
    auto listenFd = client.createsocket(kPort);
    CHECK(listenFd >= 0) << format("create TCP client failed, fd({}) < 0", listenFd);
    LOG(INFO) << format("client connect on port {}, listen fd = {}", kPort, listenFd);

    client.onConnection = [](const SocketChannelPtr& channel) {
        auto peerAddr = channel->peeraddr();
        if (channel->isConnected()) {
            LOG(INFO) << format("{} connected, connect fd = {}", peerAddr, channel->fd());

            // send time every 3s
            setInterval(3000, [channel](TimerID timerId) {
                if (channel->isConnected()) {
                    static int n{0};
                    channel->write(format("n = {}", n));
                    ++n;
                } else {
                    killTimer(timerId);
                }
            });

        } else {
            LOG(INFO) << format("{} disconnected, connect fd = {}", peerAddr, channel->fd());
        }
    };
    client.onMessage = [](const SocketChannelPtr& channel, Buffer* buf) {
        LOG(INFO) << format("receive (len={}): {}", buf->size(), static_cast<char*>(buf->data()));
        channel->write(buf);
    };
    client.onWriteComplete = [](const SocketChannelPtr& channel, Buffer* buf) {
        LOG(INFO) << format("finish write(len={}): {}", buf->size(), static_cast<char*>(buf->data()));
    };

    reconn_setting_t reConn;
    reconn_setting_init(&reConn);
    reConn.min_delay = 1000;
    reConn.max_delay = 10000;
    reConn.delay_policy = 2;
    client.setReconnect(&reConn);
    client.start();

    while (true) {
        this_thread::sleep_for(1s);
    }

    closeLog();
    return 0;
}