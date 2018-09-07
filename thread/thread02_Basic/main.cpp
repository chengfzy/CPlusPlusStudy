#include <chrono>
#include <iostream>
#include <thread>

using namespace std;

void threadTask(int n) {
    this_thread::sleep_for(chrono::seconds(n));
    cout << "hello thread " << this_thread::get_id() << " paused " << n << " seconds." << endl;
}

int main(int argc, char* argv[]) {
    thread threads[5];
    cout << "spawning 5 threads..." << endl;
    for (int i = 0; i < 5; ++i) {
        threads[i] = thread(threadTask, i + 1);
    }

    cout << "Done spawing threads. Now wait for them to join" << endl;
    for (auto& t : threads) {
        t.join();
    }
    cout << "All threads joined" << endl;

    return 0;
}
