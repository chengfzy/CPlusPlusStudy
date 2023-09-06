#include <fmt/format.h>
#include <fmt/ostream.h>
#include <glog/logging.h>
#include <boost/process.hpp>
#include <iostream>
#include <thread>
#include <common/common.hpp>

using namespace std;
using namespace fmt;
using namespace common;
using namespace boost::process;
namespace bp = boost::process;

int main(int argc, const char* argv[]) {
    initLog(argc, argv);

    // set environment
    auto env = boost::this_process::environment();
    env["PATH"].append("/home/jeffery/Documents/Code/Others/CPlusPlusStudy/buildRelease/bin");
    env["PATH"].append("/home/jeffery/Documents/Work/Code/emg_lane_mapping/buildRelease/bin");
    env["LD_LIBRARY_PATH"].append("/emg/vehicle/lib");

    string log1{"/home/jeffery/Downloads/process1.log"};
    string log2{"/home/jeffery/Downloads/process2.log"};

    string firstBin{"syntax01_move"};
    int ret = bp::system(bp::search_path(firstBin), env, std_out > log1, std_err > log1);
    LOG(INFO) << format("process1 ret: {}", ret);

    // string bin{"thread01_HelloWorld"};
    // child c(bp::search_path(bin), args = {"--url=hello"}, env, std_out > log2, std_err > log2);
    string bin{"lane_mapping"};
    auto binPath = bp::search_path(bin);
    child c(
        bp::search_path(bin),
        args = {"--url=/emg/vehicle/config/url_def_office.json", "--config=/emg/vehicle/config/LaneOptionsMapping.json",
                "--dataParams=/home/jeffery/Documents/Work/Code/emg_lane_mapping/config/"
                "SENSING_ICM42688_20220401_v0.2.x.json",
                "--device=left"},
        env, std_out > stdout, std_err > stderr);

    LOG(INFO) << format("PID: {}", c.id());
    this_thread::sleep_for(1s);
    // c.terminate();
    kill(c.id(), SIGINT);  // send Ctrl+C to kill process
    LOG(INFO) << format("terminate is running: {}", c.running());

    c.wait();
    LOG(INFO) << format("process2 exit code: {}", c.exit_code());

    this_thread::sleep_for(3s);

    closeLog();
    return 0;
}