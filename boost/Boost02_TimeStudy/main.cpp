#include <time.h>
#include <chrono>
#include <iostream>
#include "boost/date_time/posix_time/posix_time.hpp"

using namespace std;
using namespace boost;

int main(int argc, char* argv[]) {
    // time_t and tm
    {
        cout << "========================= time_t and tm =========================" << endl;

        // convert time_t to tm
        time_t t0 = time(nullptr);
        cout << "t0 = " << t0 << endl;
        tm* t1 = gmtime(&t0);
        cout << "t1 = " << asctime(t1);
        //        delete t1;

        // convert tm to time_t
        tm t3;
        t3.tm_year = 118;
        t3.tm_mon = 0;
        t3.tm_mday = 1;
        t3.tm_hour = 0;
        t3.tm_min = 0;
        t3.tm_sec = 0;
        time_t t4 = mktime(&t3);
        cout << "t3 = " << asctime(&t3);
        cout << "t4 = " << t4 << endl;
    }

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

    // 1970 time
    boost::posix_time::ptime t1970(boost::gregorian::date(1970, 1, 1));
    double clockFrom1970 = (dateTime0 - t1970).total_milliseconds() / 1000.0;  // ms
    cout << "tick from 1970 = " << clockFrom1970 << " ms";

    return 0;
}
