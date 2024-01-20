#include <fmt/format.h>
#include <fmt/ostream.h>
#include <fmt/std.h>
#include <chrono>
#include <common/common.hpp>
#include <future>
#include <iostream>
#include <string>
#include <thread>

using namespace std;
using namespace fmt;
using namespace common;

// count down taking a second for each value
int countdown(int from, int to) {
    cout << format("thread ID for countdown: {}", this_thread::get_id()) << endl;
    for (int i = from; i != to; --i) {
        cout << i << " " << flush;
        this_thread::sleep_for(200ms);
    }
    cout << "Finished" << endl;
    return from - to;
}

// simple example 01
void simple01() {
    packaged_task<int(int, int)> task(countdown);
    future<int> ret = task.get_future();
    thread th(move(task), 10, 0);
    int value = ret.get();
    cout << format("This countdown lasted for {} seconds", value) << endl;
    th.join();
}

// simple example 02
void simple02() {
    packaged_task<int(int, int)> task(countdown);
    future<int> ret = task.get_future();
    task(10, 0);
    int value = ret.get();
    cout << format("This countdown lasted for {} seconds", value) << endl;
}

int main(int argc, char* argv[]) {
    cout << format("main thread ID: {}", this_thread::get_id()) << endl;

    simple01();
    simple02();

    return 0;
}
