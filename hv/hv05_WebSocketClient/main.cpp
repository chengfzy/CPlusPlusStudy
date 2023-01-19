#include <fmt/format.h>
#include <hv/WebSocketClient.h>
#include <boost/lexical_cast.hpp>
#include <common/common.hpp>
#include <iostream>

using namespace std;
using namespace fmt;
using namespace common;
using namespace hv;

int main(int argc, const char* argv[]) {
    initLog(argc, argv);

    const string url{"ws://127.0.0.1:8800"};
    WebSocketClient ws;
    ws.onopen = [&]() {
        LOG(INFO) << "on open";
        ws.send("hello");
    };

    ws.onmessage = [](const string& msg) { LOG(INFO) << format("on message, receive {}", msg); };
    ws.onclose = []() { LOG(INFO) << format("on close"); };

    reconn_setting_t reConn;
    reconn_setting_init(&reConn);
    reConn.min_delay = 1000;
    reConn.max_delay = 10000;
    reConn.delay_policy = 2;

    ws.open(url.c_str());

    while (true) {
        this_thread::sleep_for(1s);
    }

    closeLog();
    return 0;
}