#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>

using namespace std;

mutex mtx;              // mutex lock
condition_variable cv;  // condition variable
int cargo{0};

bool shipmentAvailable() { return cargo != 0; }

// consumer thread
void consume(int n) {
    for (int i = 0; i < n; ++i) {
        unique_lock<mutex> lck(mtx);
        cv.wait(lck, shipmentAvailable);
        cout << "customer: cargo = " << cargo << endl;
        cargo = 0;
        cout << "customer reset: cargo = " << cargo << endl;
    }
}

int main(int argc, char* argv[]) {
    // consumer thread
    thread consumerThread(consume, 10);

    // producer thread, produce 10 product
    for (int i = 0; i < 10; ++i) {
        while (shipmentAvailable()) {
            this_thread::yield();
        }

        unique_lock<mutex> lck(mtx);
        cargo = i + 1;
        cout << "produces: cargo = " << cargo << endl;
        cv.notify_one();
    }

    consumerThread.join();

    return 0;
}
