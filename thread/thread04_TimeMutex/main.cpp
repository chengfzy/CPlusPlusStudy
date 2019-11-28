#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>

using namespace std;

timed_mutex mtx;

void fireworks() {
    // waiting to get a lock: each thread prints "-" every 200ms
    while (!mtx.try_lock_for(chrono::milliseconds(200))) {
        cout << "-";
    }

    // got a lock, wait for 1s, and then this thread prints "*"
    this_thread::sleep_for(chrono::milliseconds(1000));
    cout << "*" << endl;
    mtx.unlock();
}

int main(int argc, char* argv[]) {
    thread threads[10];
    // spawn 10 threads
    for (int i = 0; i < 10; ++i) {
        threads[i] = thread(fireworks);
    }

    for (auto& t : threads) {
        t.join();
    }

    return 0;
}
