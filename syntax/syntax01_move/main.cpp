#include <fmt/ranges.h>
#include <future>
#include <iostream>
#include <thread>
#include <common/common.hpp>

using namespace std;
using namespace fmt;
using namespace common;

class MyStr {
  public:
    explicit MyStr(string&& str) : str_(str) { cout << "move constructor" << endl; }
    explicit MyStr(const string& str) : str_(str) { cout << "copy constructor" << endl; }

    const string& str() const { return str_; }

  private:
    string str_;
};

void carefullyCase01() {
    cout << Section("Carefully Usage 01") << endl;
    vector<double> data;
    double x{0};
    for (size_t i = 0; i < 10; ++i) {
        x = i * 2;
        data.emplace_back(move(x));
    }

    cout << format("data = {}", data) << endl;
}

int main(int argc, char* argv[]) {
    {
        cout << Section("Test 01") << endl;
        string a{"data"};
        string&& b = move(a);
        cout << "a: " << a << ", b: " << b << endl;
    }

    {
        cout << Section("Test 02") << endl;
        string a{"data"};
        string b{move(a)};
        cout << "a: " << a << ", b: " << b << endl;
    }

    {
        cout << Section("Test 03") << endl;
        string a{"data"};
        MyStr myStr(a);
        cout << "a: " << a << ", myStr: " << myStr.str() << endl;
    }

    {
        cout << Section("Test 04") << endl;
        MyStr myStr("data");
        cout << "myStr: " << myStr.str() << endl;
    }

    carefullyCase01();

    return 0;
}