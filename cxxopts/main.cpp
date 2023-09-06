#include <fmt/format.h>
#include <fmt/ranges.h>
#include <cxxopts.hpp>
#include <iostream>
#include <common/common.hpp>

using namespace std;
using namespace fmt;
using namespace common;

int main(int argc, char* argv[]) {
    // options
    cxxopts::Options options(argv[0], "Study code about cxxopts");
    bool print{false};
    options.positional_help("file output").show_positional_help();
    // clang-format off
    options.add_options()
        ("f,file", "file name", cxxopts::value<string>()->default_value("~/Documents/input.txt"))
        ("o,output", "output file", cxxopts::value<string>(), "OUT")
        ("print", "print information", cxxopts::value(print))
        ("l,list", "input list", cxxopts::value<vector<float>>())
        ("h,help", "print usage");
    // clang-format on
    // add group
    options.add_options("Set")("n,num", "number", cxxopts::value<int>()->implicit_value("8"));

    options.parse_positional({"file", "output"});
    auto result = options.parse(argc, argv);
    // print help
    if (result.count("help")) {
        cout << options.help({"", "Set"}) << endl;
        return 0;
    }
    // with default option
    string file = result["file"].as<string>();
    cout << format("file: {}", file) << endl;

    // with argument description
    string output = result["output"].as<string>();
    cout << format("output: {}", output) << endl;

    // save to variable directly, and bool option
    cout << format("print: {}", print) << endl;

    // list
    vector<float> list = result["list"].as<vector<float>>();
    cout << format("list: {}", list) << endl;

    // with implicit value
    int num = result["num"].as<int>();
    cout << format("num: {}", num) << endl;

    return 0;
}
