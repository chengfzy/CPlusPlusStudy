#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>

using namespace std;

mutex mtx;              // mutex lock
condition_variable cv;  // condition variable
bool ready{false};      // flag

void printId(int id) {
    unique_lock<mutex> lck(mtx);

    while (!ready) {   // wait if flag is not true
        cv.wait(lck);  // block, wake up until flag is true
    }
    cout << "thread " << id << endl;
}

void go() {
    unique_lock<mutex> lck(mtx);
    ready = true;
    cv.notify_all();
}

int main(int argc, char* argv[]) {
    thread threads[10];
    // spawn 10 threads
    for (int i = 0; i < 10; ++i) {
        threads[i] = thread(printId, i);
    }

    cout << "10 thread ready to race..." << endl;
    go();

    for (auto& t : threads) {
        t.join();
    }

    return 0;
}
