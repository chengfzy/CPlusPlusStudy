/**
 * @brief Find value y1, y2 in data to ensure y1 <= x <= y2
 *
 */

#include <fmt/format.h>
#include <algorithm>
#include <iostream>
#include <common/common.hpp>

using namespace std;
using namespace fmt;
using namespace common;

template <typename T>
void method01(const vector<int>& data, const T& x) {
    auto it2 = lower_bound(data.begin(), data.end(), x, [](int v, const T& s) { return v <= s; });
    if (it2 == data.begin() || it2 == data.end()) {
        LOG(ERROR) << format("method01: cannot find, x = {}", x);
        return;
    }
    auto it1 = prev(it2);
    LOG(INFO) << format("method01: x = {}, y1 = {}, y2 = {}", x, *it1, *it2);
}

template <typename T>
void method02(const vector<int>& data, const T& x) {
    auto it2 = upper_bound(data.begin(), data.end(), x, [](const T& s, int v) { return s < v; });
    if (it2 == data.begin() || it2 == data.end()) {
        LOG(ERROR) << format("method02: cannot find, x = {}", x);
        return;
    }
    auto it1 = prev(it2);
    LOG(INFO) << format("method02: x = {}, y1 = {}, y2 = {}", x, *it1, *it2);
}

template <typename T>
void method03(const vector<int>& data, const T& x) {
    auto it1 = lower_bound(data.rbegin(), data.rend(), x, [](int v, const T& s) { return v > s; });
    if (it1 == data.rbegin() || it1 == data.rend()) {
        LOG(ERROR) << format("method03: cannot find, x = {}", x);
        return;
    }
    auto it2 = prev(it1);
    LOG(INFO) << format("method03: x = {}, y1 = {}, y2 = {}", x, *it1, *it2);
}
template <typename T>
void method04(const vector<int>& data, const T& x) {
    auto it1 = upper_bound(data.rbegin(), data.rend(), x, [](const T& s, int v) { return s >= v; });
    if (it1 == data.rbegin() || it1 == data.rend()) {
        LOG(ERROR) << format("method04: cannot find, x = {}", x);
        return;
    }
    auto it2 = prev(it1);
    LOG(INFO) << format("method04: x = {}, y1 = {}, y2 = {}", x, *it1, *it2);
}

int main(int argc, const char* argv[]) {
    initLog(argc, argv);

    vector<int> data{0, 1, 2, 3, 4, 5, 5, 6, 7, 8, 9, 10};
    LOG(INFO) << "data in middle";
    method01(data, 4.5);
    method01(data, 5);

    method02(data, 4.5);
    method02(data, 5);

    method03(data, 4.5);
    method03(data, 5);

    method04(data, 4.5);
    method04(data, 5);

    LOG(INFO) << "data in boarder";
    method01(data, 0);
    method01(data, -0.5);
    method01(data, 10);
    method01(data, 10.5);
    method02(data, 0);
    method02(data, -0.5);
    method02(data, 10);
    method02(data, 10.5);
    method03(data, 0);
    method03(data, -0.5);
    method03(data, 10);
    method03(data, 10.5);
    method04(data, 0);
    method04(data, -0.5);
    method04(data, 10);
    method04(data, 10.5);

    closeLog();
    return 0;
}
