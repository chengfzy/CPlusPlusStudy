namespace common {

// Constructor
template <HeadingType Type, unsigned short SecLen>
Heading<Type, SecLen>::Heading(std::string&& text, bool breakLine) : text_(std::move(text)), breakLine_(breakLine) {
    static_assert(SecLen >= 10, "section length should >= 10");
}

// Constructor
template <HeadingType Type, unsigned short SecLen>
Heading<Type, SecLen>::Heading(const std::string& text, bool breakLine) : text_(text), breakLine_(breakLine) {
    static_assert(SecLen >= 10, "section length should >= 10");
}

// Print heading for Title
template <unsigned short SecLen>
std::ostream& operator<<(std::ostream& os, const Heading<HeadingType::Title, SecLen>& info) {
    if (info.breakLine_) {
        os << std::endl;
    }

    int len = std::max(SecLen - 2, static_cast<int>(info.text_.size() + 10));
    os << fmt::format(fmt::emphasis::bold | fg(fmt::color::cyan), "╔{:═^{}}╗", "", len) << std::endl;
    os << fmt::format(fmt::emphasis::bold | fg(fmt::color::cyan), "║{:^{}}║", "", len) << std::endl;
    os << fmt::format(fmt::emphasis::bold | fg(fmt::color::cyan), "║{:^{}}║", " " + info.text_ + " ", len) << std::endl;
    os << fmt::format(fmt::emphasis::bold | fg(fmt::color::cyan), "║{:^{}}║", "", len) << std::endl;
    os << fmt::format(fmt::emphasis::bold | fg(fmt::color::cyan), "╚{:═^{}}╝", "", len) << std::endl;

    return os;
}

// Print heading for Section
template <unsigned short SecLen>
std::ostream& operator<<(std::ostream& os, const Heading<HeadingType::Section, SecLen>& info) {
    if (info.breakLine_) {
        os << std::endl;
    }

    if (info.text_.empty()) {
        os << fmt::format(fg(fmt::color::cyan), "{:═^{}}", "", SecLen) << std::endl;
    } else {
        os << fmt::format(fg(fmt::color::cyan), "{:═^{}}", " " + info.text_ + " ",
                          std::max(SecLen, static_cast<unsigned short>(info.text_.size() + 12)))
           << std::endl;
    }

    return os;
}

// Print heading for SubSection
template <unsigned short SecLen>
std::ostream& operator<<(std::ostream& os, const Heading<HeadingType::SubSection, SecLen>& info) {
    if (info.breakLine_) {
        os << std::endl;
    }

    if (info.text_.empty()) {
        os << fmt::format(fg(fmt::color::cyan), "{:━^{}}", "", SecLen) << std::endl;
    } else {
        os << fmt::format(fg(fmt::color::cyan), "{:━^{}}", " " + info.text_ + " ",
                          std::max(SecLen, static_cast<unsigned short>(info.text_.size() + 12)))
           << std::endl;
    }

    return os;
}

// Print heading for Paragraph
template <unsigned short SecLen>
std::ostream& operator<<(std::ostream& os, const Heading<HeadingType::Paragraph, SecLen>& info) {
    if (info.breakLine_) {
        os << std::endl;
    }

    if (info.text_.empty()) {
        os << fmt::format(fg(fmt::color::cyan), "{:─^{}}", "", SecLen / 1.5) << std::endl;
    } else {
        os << fmt::format(fg(fmt::color::cyan), "{:─^{}}", " " + info.text_ + " ",
                          std::max(static_cast<int>(SecLen / 1.5f), static_cast<int>(info.text_.size() + 12)))
           << std::endl;
    }

    return os;
}

}  // namespace common