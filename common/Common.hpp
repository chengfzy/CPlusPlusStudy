#pragma once
#include <fstream>
#include <iostream>
#include <string>

/**
 * @brief Get section string
 * @param title     Section title(name)
 * @param breakLine New line to print this section
 * @return Section string
 */
std::string section(const std::string& title, bool breakLine = true) {
    std::string str;
    if (breakLine) {
        str += "\n";
    }
    if (title.empty()) {
        return str + std::string(120, '=');
    }

    std::string fillStr((120 - title.size()) / 2, '=');
    return str + fillStr + " " + title + " " + fillStr;
}

/**
 * @brief Get subsection string
 * @param title     Subsection title(name)
 * @param breakLine New line to print this subsection
 * @return Subsection string
 */
std::string subSection(const std::string& title, bool breakLine = true) {
    std::string str;
    if (breakLine) {
        str += "\n";
    }
    if (title.empty()) {
        return str + std::string(120, '-');
    }

    std::string fillStr((120 - title.size()) / 2, '-');
    return str + fillStr + " " + title + " " + fillStr;
}

/**
 * @brief Get paragraph string
 * @param title Paragraph title(name)
 * @return Paragraph string
 */
std::string paragraph(const std::string& title) {
    if (title.empty()) {
        return std::string(20, '-');
    }
    std::string fillStr((25 - title.size()) / 2, '=');
    return fillStr + " " + title + " " + fillStr;
}
