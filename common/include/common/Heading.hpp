#pragma once
#include <fmt/color.h>
#include <fmt/format.h>
#include <ostream>
#include <string>
#include <type_traits>

namespace common {

/**
 * @brief Heading type
 */
enum class HeadingType {
    Title,
    Section,
    SubSection,
    Paragraph,
};

/**
 * @brief The default setting for whether to add a break line for heading type
 * @param type Heading type
 * @return True for add a break line, otherwise return false
 */
inline bool defaultBreakLine(HeadingType type) { return type != HeadingType::Paragraph; }

/**
 * @brief Heading to print or show some formatted text
 * @tparam Type     Heading type
 * @tparam SecLen   Print length for section and subsection type, and the length of paragraph will be SecLen / 1.5
 */
template <HeadingType Type = HeadingType::Section, unsigned short SecLen = 100>
class Heading {
  public:
    /**
     * @brief Constructor
     * @param text      Heading text
     * @param breakLine Flag to indict whether add a new line to show this info
     */
    explicit Heading(std::string&& text, bool breakLine = defaultBreakLine(Type));

    /**
     * @brief Constructor
     * @param text      Heading text
     * @param breakLine Flag to indict whether add a new line to show this info
     */
    explicit Heading(const std::string& text, bool breakLine = defaultBreakLine(Type));

    /**
     * @brief Destructor
     */
    ~Heading() = default;

  public:
    /**
     * @brief Print heading for Title
     * @param os    Output stream
     * @param info  Heading info
     * @return Output stream
     */
    template <unsigned short SecLen_>
    friend std::ostream& operator<<(std::ostream& os, const Heading<HeadingType::Title, SecLen_>& info);

    /**
     * @brief Print heading for Section
     * @param os    Output stream
     * @param info  Heading info
     * @return Output stream
     */
    template <unsigned short SecLen_>
    friend std::ostream& operator<<(std::ostream& os, const Heading<HeadingType::Section, SecLen_>& info);

    /**
     * @brief Print heading for SubSection
     * @param os    Output stream
     * @param info  Heading info
     * @return Output stream
     */
    template <unsigned short SecLen_>
    friend std::ostream& operator<<(std::ostream& os, const Heading<HeadingType::SubSection, SecLen_>& info);

    /**
     * @brief Print heading for Paragraph
     * @param os    Output stream
     * @param info  Heading info
     * @return Output stream
     */
    template <unsigned short SecLen_>
    friend std::ostream& operator<<(std::ostream& os, const Heading<HeadingType::Paragraph, SecLen_>& info);

  private:
    std::string text_;  // heading text
    bool breakLine_;    // flag to indict whether add a break line to show this heading
};

/**************************************** Type Definition ****************************************/
using Title = Heading<HeadingType::Title>;
using Section = Heading<HeadingType::Section>;
using SubSection = Heading<HeadingType::SubSection>;
using Paragraph = Heading<HeadingType::Paragraph>;

}  // namespace common

#include "implementation/Heading.hpp"