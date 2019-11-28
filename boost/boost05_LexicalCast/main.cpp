#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <exception>
#include <iostream>

using namespace std;

int main(int argc, char* argv[]) {
    try {
        string s = boost::lexical_cast<string>(123);
        cout << "string: " << s << endl;
        double d = boost::lexical_cast<double>(s);
        cout << "double: " << d << endl;

        string intStr{" 123"};
        int x = boost::lexical_cast<int>(boost::algorithm::trim_copy(intStr));
        cout << "int: " << x << endl;

        int i = boost::lexical_cast<int>("abc");
        cout << "int: " << i << endl;

    } catch (const exception& e) {
        cerr << e.what() << endl;
    }

    return 0;
}
