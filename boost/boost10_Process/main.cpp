#include <fmt/format.h>
#include <fmt/ostream.h>
#include <glog/logging.h>
#include <boost/process.hpp>
#include <iostream>
#include "common/common.hpp"

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

    string log1{"/home/jeffery/Downloads/tempCode/process1.log"};
    string log2{"/home/jeffery/Downloads/tempCode/process2.log"};

    string firstBin{"syntax01_move"};
    int ret = bp::system(bp::search_path(firstBin), env, std_out > log1, std_err > log1);
    LOG(INFO) << format("process1 ret: {}", ret);

    FLAGS_log_dir = "/home/jeffery/Downloads/tempCode";
    string bin{"thread01_HelloWorld"};
    child c(bp::search_path(bin), args = {"--url=hello"}, env, std_out > log2, std_err > log2);

    LOG(INFO) << format("PID: {}", c.id());
    LOG(INFO) << format("is running: {}", c.running());

    c.wait();
    LOG(INFO) << format("process1 exit code: {}", c.exit_code());

    closeLog();
    return 0;
}