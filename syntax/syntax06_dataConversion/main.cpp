#include <iostream>
#include <common/common.hpp>

using namespace std;
using namespace fmt;
using namespace common;

int main(int argc, const char* argv[]) {
    initLog(argc, argv);

    uint8_t raw[8] = {45, 230, 35, 131, 109, 242, 14, 23};
    LOG(INFO) << format("raw = {}", join(raw, ","));
    LOG(INFO) << format("raw address: {}", ptr(raw));

    uint64_t t1{0}, t2{0};
    for (size_t i = 0; i < 8; ++i) {
        t1 += static_cast<uint64_t>(raw[i]) << (8 * i);
        t2 += static_cast<uint64_t>(raw[i]) << (8 * (7 - i));
    }
    uint64_t t3{0};
    memcpy(&t3, raw, 8);
    uint64_t t4 = *((uint64_t*)raw);
    uint64_t t5 = *reinterpret_cast<uint64_t*>(raw);
    uint64_t t6 = reinterpret_cast<uint64_t>(raw);
    LOG(INFO) << format("t1 = {}, t2 = {}, t3 = {}, t4 = {}, t5 = {}, t6 = {}", t1, t2, t3, t4, t5, t6);

    closeLog();
    return 0;
}