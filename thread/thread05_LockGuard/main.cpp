#include <chrono>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>

using namespace std;

mutex mtx;

void printEven(int x) {
    if (x % 2 == 0) {
        cout << x << " is even" << endl;
    } else {
        throw logic_error("non even");
    }
}

void printThreadId(int id) {
    try {
        // using a local lock_guard to lock mtx guarantees unlocking on destruction exception
        lock_guard<mutex> lck(mtx);
        printEven(id);
    } catch (logic_error& e) {
        cout << "[exception caught] " << e.what() << endl;
    }
}

int main(int argc, char* argv[]) {
    thread threads[10];
    for (int i = 0; i < 10; ++i) {
        threads[i] = thread(printThreadId, i + 1);
    }

    for (auto& t : threads) {
        t.join();
    }

    return 0;
}
