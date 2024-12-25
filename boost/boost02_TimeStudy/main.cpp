#include <fmt/chrono.h>
#include <fmt/format.h>
#include <fmt/ostream.h>
#include <time.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <chrono>
#include <iostream>
#include <common/common.hpp>

using namespace std;
using namespace fmt;
using namespace common;
using namespace boost;

int main(int argc, const char* argv[]) {
    initLog(argc, argv);

    // time_t and tm
    {
        LOG(INFO) << Section("time_t and tm");

        // convert time_t to tm
        time_t t0 = time(nullptr);
        LOG(INFO) << format("t0 = {}", t0);
        tm* t1 = gmtime(&t0);
        LOG(INFO) << format("t1 = {}", asctime(t1));
        // delete t1;

        // convert tm to time_t
        tm t3;
        t3.tm_year = 118;
        t3.tm_mon = 0;
        t3.tm_mday = 1;
        t3.tm_hour = 0;
        t3.tm_min = 0;
        t3.tm_sec = 0;
        time_t t4 = mktime(&t3);
        LOG(INFO) << format("t3 = {}", asctime(&t3));
        LOG(INFO) << format("t4 = {}", t4);
    }

    // chrono Time
    {
        LOG(INFO) << Section("chrono Time");
        chrono::steady_clock::time_point t0 = chrono::steady_clock::now();
        double sum{0};
        for (int i = 0; i < 100000; ++i) {
            sum += i * 8;
        }
        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        auto runTime = chrono::duration_cast<chrono::microseconds>(t1 - t0);
        LOG(INFO) << format("Run Time = {} ms", runTime.count());
        chrono::microseconds d1(10);
        LOG(INFO) << format("duration = {}", d1.count());

        // time add
        auto t2 = t0 + chrono::microseconds(1000);
        LOG(INFO) << format("t0 = {}", t0.time_since_epoch());
        LOG(INFO) << format("t2 = {}", t2.time_since_epoch());

        // from time count
        chrono::steady_clock::time_point t3(chrono::nanoseconds(t2.time_since_epoch().count()));
        LOG(INFO) << format("t3 = {}", t3.time_since_epoch());
    }

    // Boost-Time
    {
        LOG(INFO) << Section("boost Time");
        boost::posix_time::ptime dateTime0 = boost::posix_time::microsec_clock::local_time();
        LOG(INFO) << format("dateTime0 = {}", fmt::streamed(dateTime0));
        boost::gregorian::date date(2018, 1, 18);
        boost::posix_time::time_duration time(10, 20, 12, 134.61);  // float fractional second is unnecessary
        boost::posix_time::ptime dateTime(date, time);
        LOG(INFO) << format("datetime = {}", fmt::streamed(dateTime));
        LOG(INFO) << format("time = {}", fmt::streamed(time));
        // add time
        time += boost::posix_time::seconds(20);
        LOG(INFO) << format("time = {}", fmt::streamed(time));
        time += boost::posix_time::milliseconds(56);
        LOG(INFO) << format("time = {}", fmt::streamed(time));
        // format date
        auto t0 = boost::posix_time::microsec_clock::local_time();
        LOG(INFO) << format("current time t0: {}-{}-{} {}:{}:{}.{}", fmt::streamed(t0.date().year()),
                            fmt::streamed(t0.date().month()), fmt::streamed(t0.date().day()),
                            fmt::streamed(t0.time_of_day().hours()), fmt::streamed(t0.time_of_day().minutes()),
                            fmt::streamed(t0.time_of_day().seconds()),
                            fmt::streamed(t0.time_of_day().fractional_seconds()));
        // format chrono
        auto t1 = chrono::system_clock::now();
        LOG(INFO) << format("current time t1: {:%Y-%m-%d_%H:%M:%S}", t1);

        // time since epoch(1970.1.1)
        boost::posix_time::ptime t1970(boost::gregorian::date(1970, 1, 1));
        auto timeCount = (dateTime0 - t1970).total_nanoseconds();  // ns
        LOG(INFO) << format("tick from 1970 = {} ns", timeCount);
    }

    closeLog();
    return 0;
}
