#include <chrono>
#include <iostream>
#include "boost/date_time/posix_time/posix_time.hpp"

using namespace std;
using namespace boost;

int main(int argc, char* argv[]) {
    // Chrono Time
    cout << "========================= Chrono-Time =========================" << endl;
    chrono::steady_clock::time_point t0 = chrono::steady_clock::now();
    double sum{0};
    for (int i = 0; i < 100000; ++i) {
        sum += i * 8;
    }
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    auto runTime = chrono::duration_cast<chrono::milliseconds>(t1 - t0);
    cout << "Run Time = " << runTime.count() << endl;
    chrono::microseconds d1(10);
    cout << "duration = " << d1.count() << endl;

    // Boost-Time
    cout << "========================= Boost-Time =========================" << endl;
    boost::posix_time::ptime dateTime0 = boost::posix_time::microsec_clock::universal_time();
    cout << "dateTime0 = " << dateTime0 << endl;
    boost::posix_time::time_duration time(10, 20, 12, 134);
    boost::gregorian::date date(2018, 1, 18);
    boost::posix_time::ptime dateTime(date, time);
    cout << "datetime = " << dateTime << endl;
    cout << "time = " << time << endl;
    time += boost::posix_time::milliseconds(56);
    cout << "time = " << time << endl;

    return 0;
}
