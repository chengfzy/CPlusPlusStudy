namespace common {

// Constructor
template <HeadingType Type, unsigned short ParLen>
Heading<Type, ParLen>::Heading(std::string&& text, bool breakLine) : text_(std::move(text)), breakLine_(breakLine) {}

// Constructor
template <HeadingType Type, unsigned short ParLen>
Heading<Type, ParLen>::Heading(const std::string& text, bool breakLine) : text_(text), breakLine_(breakLine) {}

// Print heading
template <HeadingType Type, unsigned short ParLen>
std::ostream& operator<<(std::ostream& os, const Heading<Type, ParLen>& info) {
    if (info.breakLine_) {
        os << std::endl;
    }
    unsigned int infoLen{0};
    char fillChar('=');
    switch (Type) {
        case HeadingType::Section:
            infoLen = static_cast<unsigned int>(1.5f * ParLen);
            fillChar = '=';
            break;
        case HeadingType::SubSection:
            infoLen = static_cast<unsigned int>(1.5f * ParLen);
            fillChar = '-';
            break;
        case HeadingType::Paragraph:
            infoLen = ParLen;
            fillChar = '=';
            break;
    }

    if (info.text_.empty()) {
        os << std::string(infoLen, fillChar);
    } else {
        std::string fillStr(std::max(5, static_cast<int>((infoLen - info.text_.size()) / 2)), fillChar);
        os << fillStr << " " << info.text_ << " " << fillStr;
    }
    os << std::endl;

    return os;
}

}  // namespace common