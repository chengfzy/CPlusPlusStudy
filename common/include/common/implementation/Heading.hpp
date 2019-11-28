namespace common {

// Constructor
template <HeadingType Type, unsigned short SecLen>
Heading<Type, SecLen>::Heading(std::string&& text, bool breakLine) : text_(std::move(text)), breakLine_(breakLine) {}

// Constructor
template <HeadingType Type, unsigned short SecLen>
Heading<Type, SecLen>::Heading(const std::string& text, bool breakLine) : text_(text), breakLine_(breakLine) {}

// Print heading
template <HeadingType Type, unsigned short SecLen>
std::ostream& operator<<(std::ostream& os, const Heading<Type, SecLen>& info) {
    if (info.breakLine_) {
        os << std::endl;
    }
    int len = SecLen;
    char fillChar('=');
    switch (Type) {
        case HeadingType::Section:
            fillChar = '=';
            break;
        case HeadingType::SubSection:
            fillChar = '*';
            break;
        case HeadingType::Paragraph:
            len = static_cast<int>(SecLen / 1.5f);
            fillChar = '-';
            break;
    }

    if (info.text_.empty()) {
        os << std::string(len, fillChar);
    } else {
        std::string fillStr(std::max(5, static_cast<int>((len - info.text_.size() - 1) / 2)), fillChar);
        os << fillStr << " " << info.text_ << " " << fillStr;
    }
    os << std::endl;

    return os;
}

}  // namespace common