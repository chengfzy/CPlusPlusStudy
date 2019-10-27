#include <chrono>
#include <functional>
#include <future>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

using namespace std;

class Track {
  public:
  public:
    void track(int x, const function<void(int)>& fetcher) {
        // should taken 1s to finish track method
        this_thread::sleep_for(chrono::seconds(1));

        int y = x;
        data.emplace_back(y);

        if (data.size() > 3) {
            cout << "[Tracker] begin fetch data " << *data.begin() << endl;
            fetcher(*data.begin());
            cout << "[Tracker] erase data..." << endl;
            data.erase(data.begin());
        }
    }

    void finish(const function<void(int)>& fetcher) {
        while (!data.empty()) {
            cout << "[Tracker] begin fetch retained data " << *data.begin() << endl;
            future<void> result = async(launch::async, fetcher, *data.begin());
            result.get();
            cout << "[Tracker] erase retained data..." << endl;
            data.erase(data.begin());
        }
    }

  public:
    vector<int> data;
};

class System {
  public:
    System() = default;
    ~System() = default;

  public:
    void run() {
        cout << "[System] System run..." << endl;

        thread t(&System::trackFunc, this);
        t.join();

        cout << "[System] System finish" << endl;

        cout << "[System] System data: " << endl;
        for (auto v : data) {
            cout << v << " ";
        }
        cout << endl;
        cout << "[System] Track data: " << endl;
        for (auto v : track.data) {
            cout << v << " ";
        }
        cout << endl;
    }

  public:
    void trackFunc() {
        for (int i = 0; i < 10; ++i) {
            cout << "[System] begin track data " << i << endl;
            track.track(i, bind(&System::fetch, this, placeholders::_1));
        }
        track.finish(bind(&System::fetch, this, placeholders::_1));
        cout << "[System] track finished..." << endl;
    }

    void fetch(int x) {
        cout << "[System] fetch data " << x << endl;
        // should taken 0.5 seconds to fetch data
        this_thread::sleep_for(chrono::milliseconds(500));
        data.emplace_back(x);
        cout << "[System] finished fetch" << endl;
    }

  private:
    Track track;
    vector<int> data;
};

int main(int argc, char* argv[]) {
    System system;
    system.run();

    return 0;
}
