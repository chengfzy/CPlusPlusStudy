#include <exception>
#include <iostream>
#include "boost/lexical_cast.hpp"

using namespace std;

int main(int argc, char* argv[]) {
    try {
        string s = boost::lexical_cast<string>(123);
        cout << "string: " << s << endl;
        double d = boost::lexical_cast<double>(s);
        cout << "double: " << d << endl;

        int i = boost::lexical_cast<int>("abc");
        cout << "int: " << i << endl;

    } catch (const exception& e) {
        cerr << e.what() << endl;
    }

    return 0;
}
