#include <iostream>
#include "common/common.hpp"

using namespace std;
using namespace common;

class MyStr {
  public:
    explicit MyStr(string&& str) : str_(str) { cout << "move constructor" << endl; }
    explicit MyStr(const string& str) : str_(str) { cout << "copy constructor" << endl; }

    const string& str() const { return str_; }

  private:
    string str_;
};

int main(int argc, char* argv[]) {
    {
        cout << section("Test 01") << endl;
        string a{"data"};
        string&& b = move(a);
        cout << "a: " << a << ", b: " << b << endl;
    }

    {
        cout << section("Test 02") << endl;
        string a{"data"};
        string b{move(a)};
        cout << "a: " << a << ", b: " << b << endl;
    }

    {
        cout << section("Test 03") << endl;
        string a{"data"};
        MyStr myStr(a);
        cout << "a: " << a << ", myStr: " << myStr.str() << endl;
    }

    {
        cout << section("Test 04") << endl;
        MyStr myStr("data");
        cout << "myStr: " << myStr.str() << endl;
    }

    return 0;
}