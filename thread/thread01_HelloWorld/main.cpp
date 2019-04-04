#include <chrono>
#include <iostream>
#include <thread>

using namespace std;

void threadTask() { cout << "hello thread" << endl; }

void f1(int n) {
    for (int i = 0; i < 5; ++i) {
        cout << "thread [" << n << "] executing" << endl;
        this_thread::sleep_for(chrono::milliseconds(10));
    }
}

void f2(int& n) {
    for (int i = 0; i < 5; ++i) {
        cout << "thread [2] executing" << endl;
        ++n;
        this_thread::sleep_for(chrono::milliseconds(10));
    }
}

int main(int argc, char* argv[]) {
    //    thread t(threadTask);
    //    t.join();

    int n{0};
    thread t1;                   // t1 is not a thread
    thread t2(f1, n + 1);        // pass by value
    thread t3(f2, std::ref(n));  // pass by reference
    thread t4(std::move(t3));    // t4 is now running f2(), t3 is no longer a thread

    t2.join();
    t4.join();
    cout << "Final value of n is " << n << endl;

    return 0;
}
