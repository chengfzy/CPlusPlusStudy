/**
 * @brief Some study code about atomic
 *
 * @ref
 *  1. https://www.cnblogs.com/haippy/p/3252056.html
 */

#include <fmt/format.h>
#include <atomic>
#include <iostream>
#include <sstream>
#include <thread>
#include <vector>
#include <common/common.hpp>

using namespace std;
using namespace fmt;
using namespace common;

/**
 * @brief Basic usage for atomic_flag
 *
 * 创建10个线程用于计数，率先完成计数任务的输出thread ID，后续完成计数任务的不会输出ID
 */
void basic01() {
    cout << Section("Basic01");
    atomic<bool> ready{false};
    atomic_flag winner = ATOMIC_FLAG_INIT;

    vector<thread> threads;
    cout << "spawning 10 threads that count to 1 million..." << endl;
    for (size_t i = 0; i < 10; ++i) {
        threads.emplace_back(thread([&, i]() {
            // wait for ready
            while (!ready) {
                this_thread::yield();
            }

            // count
            for (size_t j = 0; j < 100000; ++j) {
            }

            // print ID if test fail
            if (!winner.test_and_set()) {
                cout << format("thread #{} won!", i) << endl;
            }
        }));
    }

    // set ready
    this_thread::sleep_for(100ms);  // wait all thread create success
    ready = true;

    for (auto& t : threads) {
        t.join();
    }
}

/**
 * @brief 没太明白这个例子的意义在哪，感觉只是保证多次的输出不会相互交叉，类似于lock了stream的功能
 *
 */
void basic02() {
    cout << Section("Basic02");
    atomic_flag lock = ATOMIC_FLAG_INIT;
    stringstream stream;

    vector<thread> threads;
    for (size_t i = 0; i < 10; ++i) {
        threads.emplace_back(thread([&, i]() {
            while (lock.test_and_set()) {
            }
            stream << "thread #" << i << endl;
            lock.clear();
        }));
    }

    for (auto& t : threads) {
        t.join();
    }

    cout << stream.str() << endl;
}

/**
 * @brief 自旋锁
 *
 */
void basic03_SpinLock() {
    cout << Section("Basic03 Spin Lock");
    atomic_flag lock = ATOMIC_FLAG_INIT;

    vector<thread> threads;
    for (size_t i = 0; i < 10; ++i) {
        threads.emplace_back(thread([&, i]() {
            for (size_t j = 0; j < 100; ++j) {
                // acquire lock
                while (lock.test_and_set(memory_order_acquire)) {
                    // spin
                }
                cout << format("output from thread #{}", i) << endl;
                lock.clear(memory_order_release);
            }
        }));
    }

    for (auto& t : threads) {
        t.join();
    }
}

int main(int argc, char* argv[]) {
    basic01();
    basic02();
    basic03_SpinLock();

    return 0;
}
