#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG
#include <spdlog/details/os.h>
#include <spdlog/pattern_formatter.h>
#include <spdlog/spdlog.h>

using namespace std;

class MyFormatterFlag : public spdlog::custom_flag_formatter {
  public:
    void format(const spdlog::details::log_msg& log, const std::tm&, spdlog::memory_buf_t& dest) override {
        if (log.source.filename != nullptr) {
            // find the basename
            const char* basename = nullptr;
            if (sizeof(spdlog::details::os::folder_seps) == 2) {
                const char* rv = std::strrchr(log.source.filename, spdlog::details::os::folder_seps[0]);
                basename = (rv != nullptr ? rv + 1 : log.source.filename);
            } else {
                const std::reverse_iterator<const char*> begin(log.source.filename + std::strlen(log.source.filename));
                const std::reverse_iterator<const char*> end(log.source.filename);

                const auto it = std::find_first_of(begin, end, std::begin(spdlog::details::os::folder_seps),
                                                   std::end(spdlog::details::os::folder_seps) - 1);
                basename = (it != end ? it.base() : log.source.filename);
            }

            // add basename
            fmt::format_to(dest, " {}:{}", basename, log.source.line);
        }
    }

    std::unique_ptr<spdlog::custom_flag_formatter> clone() const override {
        return spdlog::details::make_unique<MyFormatterFlag>();
    }
};

int main(int argc, const char* argv[]) {
    spdlog::set_pattern("%^[%Y%m%d %H:%M:%S.%e %L%P,%t %s:%#]%$ %v");  // set global pattern

    SPDLOG_INFO("hello world");
    spdlog::info("Welcome to spdlog");
    spdlog::error("Some error message with arg: {}", 1);

    auto formatter = std::make_unique<spdlog::pattern_formatter>();
    formatter->add_flag<MyFormatterFlag>('*').set_pattern("%^[%Y%m%d %H:%M:%S.%e %L%P,%t%*]%$ %v");
    spdlog::set_formatter(std::move(formatter));

    spdlog::debug("This message should be displayed");
    spdlog::warn("Easy padding in numbers like {:08d}", 12);
    spdlog::critical("Support for int: {0:d}, hex: {0:x}, oct: {0:o}, bin: {0:b}", 42);
    SPDLOG_INFO("some information");
    SPDLOG_CRITICAL("error information");
    SPDLOG_DEBUG("some debug log {:.5f}", 1.2346799);

    spdlog::shutdown();
    return 0;
}