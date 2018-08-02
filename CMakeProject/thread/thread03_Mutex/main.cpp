#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>

using namespace std;

volatile int counter{0};  // non-atomic counter
mutex mtx;                // locks access to counter

void attemp10kIncreases() {
    for (int i = 0; i < 10000; ++i) {
        if (mtx.try_lock()) {  // only increase if currently not locked
            ++counter;
            mtx.unlock();
        }
    }
}

int main(int argc, char* argv[]) {
    thread threads[10];
    for (int i = 0; i < 10; ++i) {
        threads[i] = thread(attemp10kIncreases);
    }

    for (auto& t : threads) {
        t.join();
    }
    cout << counter << " successful increase of the counter" << endl;

    return 0;
}
