#include <any>
#include <common/common.hpp>
#include <iostream>

using namespace std;
using namespace fmt;
using namespace common;

enum class MessageType : uint8_t { MessageA = 0, MessageB };

struct MessageA {
    uint64_t frameId = 0;
    uint64_t timestamp = 0;
};

struct MessageB {
    double speed = 0;
    double posX = 0;
};

template <MessageType MsgType>
struct Helper {
    using Type = typename conditional<MsgType == MessageType::MessageA, MessageA, MessageB>::type;
};

struct MessageData {
    MessageType msgType;
    uint64_t t = 0;
    any msg;
};

int main(int argc, char* argv[]) {
    {
        cout << Section("Test 01");
        any x;
        cout << format("x.has_value() = {}", x.has_value()) << endl;
        x = 1;
        cout << format("x.has_value() = {}, x.value = {}", x.has_value(), any_cast<int>(x)) << endl;
        x = 1.5;
        // cout << format("x.has_value() = {}, x.value = {}", x.has_value(), any_cast<int>(x)) << endl;  // bad cast
        cout << format("x.has_value() = {}, x.value = {}", x.has_value(), any_cast<double>(x)) << endl;  // bad cast
    }

    {
        // using any to construct a base class, MessageA and MessageB don't has a base class, but with std::any and
        // template function, they could be treat as the same
        cout << Section("Test 01");
        vector<MessageData> data;

        data.emplace_back(MessageData{MessageType::MessageA, 1, MessageA{124, 3464576}});
        data.emplace_back(MessageData{MessageType::MessageB, 2, MessageB{124.1, 3464576.1}});
        // print
        for (auto& v : data) {
            cout << format("msg type = {}, t = {}. ", static_cast<uint8_t>(v.msgType), v.t);

            switch (v.msgType) {
                case MessageType::MessageA: {
                    auto msg = any_cast<Helper<MessageType::MessageA>::Type>(v.msg);
                    cout << format("msg.v1 = {}, msg.v2 = {}", msg.frameId, msg.timestamp) << endl;
                } break;
                case MessageType::MessageB: {
                    auto msg = any_cast<Helper<MessageType::MessageB>::Type>(v.msg);
                    cout << format("msg.v1 = {}, msg.v2 = {}", msg.speed, msg.posX) << endl;
                } break;

                default:
                    break;
            }
        }
    }

    return 0;
}