#include <spdlog/async.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

using namespace std;

int main(int argc, const char* argv[]) {
    // set default logger with multi sink
    auto consoleSink = make_shared<spdlog::sinks::stderr_color_sink_mt>();
    consoleSink->set_level(spdlog::level::info);  // console log only show warn and error message

    // file sink show all level logs, and rotate log with max 5Mb size and 3 rotated files
    auto fileSink = make_shared<spdlog::sinks::rotating_file_sink_mt>("./logs/log.log", 1024 * 1024 * 5, 3);
    fileSink->set_level(spdlog::level::trace);  // file show all level

    // create async logger with console and file sync
    spdlog::init_thread_pool(8192, 1);  // queue with 8k items and 1 backing thread
    auto asyncLogger =
        std::make_shared<spdlog::async_logger>("async", spdlog::sinks_init_list({consoleSink, fileSink}),
                                               spdlog::thread_pool(), spdlog::async_overflow_policy::overrun_oldest);
    // set it to default
    spdlog::set_default_logger(asyncLogger);

    spdlog::set_level(spdlog::level::debug);                          // set log level
    spdlog::set_pattern("%^[%Y%m%d %H:%M:%S.%e %L%P,%t:%s%#]%$ %v");  // set global pattern
    // spdlog::set_pattern("%^[%Y%m%d %H:%M:%S.%e %L%P,%t %@]%$ %v");

    spdlog::info("Welcome to spdlog");
    spdlog::error("Some error message with arg: {}", 1);
    spdlog::warn("Easy padding in numbers like {:08d}", 12);
    spdlog::critical("Support for int: {0:d}, hex: {0:x}, oct: {0:o}, bin: {0:b}", 42);

    spdlog::debug("This message should be displayed");
    SPDLOG_INFO("some information");
    SPDLOG_CRITICAL("error information");
    SPDLOG_LOGGER_DEBUG("some debug log {:.5f}", 1.2346799);

    spdlog::info("Hello World!");
    spdlog::info("This is {:.3f}", 1.23546);

    spdlog::shutdown();
    return 0;
}