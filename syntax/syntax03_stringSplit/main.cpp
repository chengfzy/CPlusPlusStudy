#include <common/common.hpp>
#include <iostream>
#include <regex>
#include <string>
#include <vector>

using namespace std;
using namespace common;

// split string method 01
vector<string> split(const string& s, const string& delimiters = " ") {
    vector<string> tokens;
    string::size_type lastPos = s.find_first_not_of(delimiters);
    string::size_type pos = s.find_first_of(delimiters, lastPos);
    while (string::npos != pos || string::npos != lastPos) {
        tokens.emplace_back(s.substr(lastPos, pos - lastPos));
        lastPos = s.find_first_not_of(delimiters, pos);
        pos = s.find_first_of(delimiters, lastPos);
    }
    return tokens;
}

int main(int argc, char* argv[]) {
    string raw{"\tabcde \t fghi jklm  nopqr   stuv\t\twxyz  "};

    cout << Section("Method01");
    vector<string> splitString01 = split(raw, "\t ");
    for (auto& v : splitString01) {
        cout << v << endl;
    }

    cout << Section("Method02");
    regex re("(\\s+)");
    vector<string> splitString02(sregex_token_iterator(raw.begin(), raw.end(), re, -1), sregex_token_iterator());
    for (auto& v : splitString02) {
        cout << v << endl;
    }

    return 0;
}