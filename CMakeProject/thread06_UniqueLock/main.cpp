#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>

using namespace std;

mutex mtx;
mutex foo, bar;

void printBlock(int n, char c) {
    // critical section (exclusive access to std::cout signaled by lifetime of lck
    unique_lock<mutex> lck(mtx);
    for (int i = 0; i < n; ++i) {
        cout << c;
    }
    cout << endl;
}

void taskA() {
    lock(foo, bar);  // simultaneous lock(prevents deadlock)
    unique_lock<mutex> lck1(foo, adopt_lock);
    unique_lock<mutex> lck2(bar, adopt_lock);
    cout << "task A" << endl;
    // unlocked automatically on destruction of lck1 and lck2
}

void taskB() {
    // foo.lock(); bar.lock()   // replaced by:
    unique_lock<mutex> lck1, lck2;
    lck1 = unique_lock<mutex>(bar, defer_lock);
    lck2 = unique_lock<mutex>(foo, defer_lock);
    lock(lck1, lck2);  // simultaneous lock (prevent deadlock)
    cout << "task B" << endl;
    // unlocked automatically on destruction of lck1 and lck2
}

int main(int argc, char* argv[]) {
    thread t1(printBlock, 50, '*');
    thread t2(printBlock, 50, '#');

    t1.join();
    t2.join();

    thread t3(taskA);
    thread t4(taskB);
    t3.join();
    t4.join();

    return 0;
}
