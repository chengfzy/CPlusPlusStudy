#pragma once
#include <string>

namespace common {

/**
 * @brief Get section string
 * @param title     Section title(name)
 * @param breakLine New line to print this section
 * @return Section string
 */
std::string section(const std::string& title, bool breakLine = true);

/**
 * @brief Get subsection string
 * @param title     Subsection title(name)
 * @param breakLine New line to print this subsection
 * @return Subsection string
 */
std::string subSection(const std::string& title, bool breakLine = true);

/**
 * @brief Get paragraph string
 * @param title Paragraph title(name)
 * @return Paragraph string
 */
std::string paragraph(const std::string& title);

}  // namespace common