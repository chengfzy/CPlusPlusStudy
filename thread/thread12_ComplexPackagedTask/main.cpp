/**
 * @brief A complex example using `packaged_task`
 *
 * 这个例子来源于一个工程应用的简化版. 比如从相机接收图像, 对图像进行JPEG压缩, 再存储为MP4文件.
 * 一般从相机接收图像时会存在一个queue中， 由于JPEG压缩比较耗时, 一般会开始多个线程进行处理,
 * 再将结果送到下一步进行MP4存储. 这里就会涉及到对JPEG文件进行排序的问题, 采用排序并不是很好的方法,
 * 特别是需要确定下一张图像什么时候才压缩完成, 中间的设计会非常复杂.
 *
 * 这里使用`packaged_task`和`future`将其问题进行简化处理. 首先, 从相机接收到原始图像后, 并不是将其存在一个简单的queue中,
 * 而是通过`packaged_task`构造一个压缩任务, 再和原始图像组合成一个`pair`, 再将其放到到queue中.
 * `packaged_task`对应的结果以`future`的形式放到另外一个queue中. 压缩线程则从task queue不停地取任务和原始图像进行处理,
 * 存储MP4的线程则挨着取JPEG结果就行, 如果没处理完, 也会等待, 这样减少了各种排序的问题.
 *
 */

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <fmt/std.h>
#include <chrono>
#include <common/common.hpp>
#include <condition_variable>
#include <future>
#include <iostream>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

using namespace std;
using namespace fmt;
using namespace common;

queue<pair<shared_ptr<packaged_task<int(int, int)>>, int>> taskQueue;  // task queue, <task, b>
mutex taskMutex;                                                       // task mutex
condition_variable taskCondition;                                      // task condition
mutex resultMutex;                                                     // result mutex
queue<future<int>> resultQueue;                                        // result queue
condition_variable resultCondition;                                    // result condition

int main(int argc, char* argv[]) {
    cout << format("main thread ID: {}", this_thread::get_id()) << endl;

    // process task thread
    vector<thread> taskThread;
    for (size_t i = 0; i < 2; ++i) {
        taskThread.emplace_back([&] {
            cout << format("task thread ID: {}", this_thread::get_id()) << endl;
            int a{2};

            while (true) {
                shared_ptr<packaged_task<int(int, int)>> task;
                int b{0};
                {
                    // get task
                    unique_lock<mutex> lock(taskMutex);
                    taskCondition.wait(lock, [&] { return !taskQueue.empty(); });
                    tie(task, b) = move(taskQueue.front());
                    taskQueue.pop();
                }

                // do task
                (*task)(a, b);

                // notify result
                resultCondition.notify_one();
            }
        });
    }

    // get result thread
    thread resultThread([&]() {
        cout << format("result thread ID: {}", this_thread::get_id()) << endl;
        int a{2};

        while (true) {
            future<int> result;
            {
                // get result
                unique_lock<mutex> lock(resultMutex);
                resultCondition.wait(lock, [&] { return !resultQueue.empty(); });
                result = move(resultQueue.front());
                resultQueue.pop();
            }

            // print result
            cout << format("result = {}", result.get()) << endl;
        }
    });

    // main thread: create task
    for (int i = 0; i < 10; ++i) {
        auto task = make_shared<packaged_task<int(int, int)>>([&](int a, int b) {
            cout << format("task thread ID: {}", this_thread::get_id()) << endl;
            cout << format("a = {}, b = {}", a, b) << endl;
            this_thread::sleep_for(200ms);
            return a + b;
        });

        // add task
        {
            unique_lock<mutex> lock(taskMutex);
            taskQueue.emplace(make_pair(task, i));
            taskCondition.notify_one();
        }

        // add result
        {
            unique_lock<mutex> lock(resultMutex);
            resultQueue.emplace(task->get_future());
        }
    }

    if (resultThread.joinable()) {
        resultThread.join();
    }

    return 0;
}
