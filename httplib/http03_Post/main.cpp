#include <fmt/format.h>
#include <fmt/ranges.h>
#include <httplib.h>
#include <iostream>
#include <thread>

using namespace std;
using namespace fmt;
using namespace httplib;

int main(int argc, const char* argv[]) {
    Client client("http://101.205.3.41:8001");

    ifstream fs("./data/test.jpg");
    stringstream buffer;
    buffer << fs.rdbuf();
    auto str = buffer.str();

    MultipartFormDataItems items = {{"file", str, "file.bin", ""}};
    auto res = client.Post("/abnormal/upload", items);
    cout << res->body << endl;

    return 0;
}