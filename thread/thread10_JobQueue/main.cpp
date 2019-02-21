#include <chrono>
#include <future>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include "JobQueue.hpp"

using namespace std;

int main(int argc, char* argv[]) {
    JobQueue<int> jobQueue(5);  // job queue with size = 5

    // multiple consumer
    vector<unique_ptr<thread>> consumerThreads;
    for (size_t i = 0; i < 3; ++i) {
        consumerThreads.emplace_back(new thread([&]() {
            while (true) {
                auto job = jobQueue.pop();
                if (job.isValid()) {
                    this_thread::sleep_for(chrono::milliseconds(100));
                    cout << "[" << this_thread::get_id() << "] consumer receive: " << job.data() << endl;
                } else {
                    cout << "[" << this_thread::get_id() << "] finish consume" << endl;
                    break;
                }
            }
        }));
    }

    // producer
    for (int i = 0; i < 20; ++i) {
        this_thread::sleep_for(chrono::milliseconds(10));
        jobQueue.push(i);
        cout << "[" << this_thread::get_id() << "] add value: " << i << endl;
    }
    cout << "[" << this_thread::get_id() << "] finish produce" << endl;

    // wait job finish
    jobQueue.wait();
    jobQueue.stop();

    // wait threads
    for (auto& thread : consumerThreads) {
        if (thread->joinable()) {
            thread->join();
        }
    }

    return 0;
}
