#include <fmt/format.h>
#include <hv/EventLoop.h>
#include <hv/WebSocketServer.h>
#include <boost/lexical_cast.hpp>
#include <common/common.hpp>
#include <iostream>

using namespace std;
using namespace fmt;
using namespace common;
using namespace hv;

int main(int argc, const char* argv[]) {
    initLog(argc, argv);

    constexpr int kPort{8800};
    WebSocketService ws;
    ws.onopen = [](const WebSocketChannelPtr& channel, const HttpRequestPtr& req) {
        LOG(INFO) << format("on open, get {}", req->Path());
        // send every 1s
        setInterval(1000, [&](TimerID id) {
            if (channel->isConnected()) {
                static int m{0};
                channel->send(format("m = {}", m++));
            } else {
                killTimer(id);
            }
        });
    };

    ws.onmessage = [](const WebSocketChannelPtr& channel, const string& msg) {
        LOG(INFO) << format("on message, receive {}", msg);
    };
    ws.onclose = [](const WebSocketChannelPtr& channel) { LOG(INFO) << format("on close"); };

    websocket_server_t server;
    server.port = kPort;
    server.ws = &ws;
    websocket_server_run(&server);

    closeLog();
    return 0;
}