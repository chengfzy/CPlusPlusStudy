#include <fmt/format.h>
#include <iostream>
#include <optional>
#include <common/common.hpp>

using namespace std;
using namespace fmt;
using namespace common;

int main(int argc, char* argv[]) {
    optional<int> x1(125);
    cout << format("x1.has_value() = {}, value = {}", x1.has_value(), x1.value()) << endl;
    // change data
    x1.emplace(356);
    cout << format("x1.value = {}", x1.value()) << endl;
    // reset
    x1.reset();
    cout << format("x1.has_value() = {}", x1.has_value()) << endl;

    // construct with exist data
    int y2{2445};
    optional<int> x2(y2);
    cout << format("y2 = {}", y2) << endl;
    cout << format("x2.has_value() = {}, value = {}", x2.has_value(), x2.value()) << endl;

    return 0;
}