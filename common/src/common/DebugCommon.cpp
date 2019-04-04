#include "common/DebugCommon.h"
#include <sstream>
#include <string>

using namespace std;

namespace common {

// Get section string
string section(const string& title, bool breakLine) {
    stringstream strStream;
    if (breakLine) {
        strStream << endl;
    }
    if (title.empty()) {
        strStream << string(120, '=');
    } else {
        string fillStr((120 - title.size()) / 2, '=');
        strStream << fillStr << " " << title << " " << fillStr;
    }
    return strStream.str();
}

// Get subsection string
string subSection(const string& title, bool breakLine) {
    stringstream strStream;
    if (breakLine) {
        strStream << endl;
    }
    if (title.empty()) {
        strStream << string(120, '-');
    } else {
        string fillStr((120 - title.size()) / 2, '-');
        strStream << fillStr << " " << title << " " << fillStr;
    }
    return strStream.str();
}

// Get paragraph string
string paragraph(const string& title) {
    if (title.empty()) {
        return string(60, '=');
    }
    string fillStr((60 - title.size()) / 2, '=');
    return fillStr + " " + title + " " + fillStr;
}

}  // namespace common