#include <boost/tokenizer.hpp>
#include <iostream>
#include <common/common.hpp>

using namespace std;
using namespace common;

// use char separator
void teste01_basic() {
    cout << endl << Section("Use Char Separator") << endl;
    using tokenizer = boost::tokenizer<boost::char_separator<char>>;
    string s = "Boost C++ Libraries; My CSV Code";
    boost::char_separator<char> sep(" ", "+", boost::keep_empty_tokens);  // specify char separator
    tokenizer tok(s, sep);
    for (tokenizer::iterator it = tok.begin(); it != tok.end(); ++it) {
        cout << *it << endl;
    }
}

// parse CSV files
void test02_parseCsv() {
    cout << endl << Section("Parse CSV Files") << endl;
    using tokenizer = boost::tokenizer<boost::escaped_list_separator<char>>;
    string s = "Boost, \"C++ Libraries\", My CSV Code";
    boost::char_separator<char> sep(" ", "+", boost::keep_empty_tokens);  // specify char separator
    tokenizer tok(s);
    for (tokenizer::iterator it = tok.begin(); it != tok.end(); ++it) {
        cout << *it << endl;
    }
}

// parse CSV files
void test03_offsetSeparator() {
    cout << endl << Section("Offset Separator") << endl;
    using tokenizer = boost::tokenizer<boost::offset_separator>;
    string s = "Boost_C++_Libraries";
    int offsets[] = {5, 5, 9};
    boost::offset_separator sep(offsets, offsets + 3);  // specify char separator
    tokenizer tok(s, sep);
    for (tokenizer::iterator it = tok.begin(); it != tok.end(); ++it) {
        cout << *it << endl;
    }
}

int main(int argc, char* argv[]) {
    teste01_basic();
    test02_parseCsv();
    test03_offsetSeparator();

    return 0;
}
