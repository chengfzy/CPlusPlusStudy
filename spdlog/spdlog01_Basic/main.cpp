#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

using namespace std;

int main(int argc, const char* argv[]) {
    spdlog::set_level(spdlog::level::debug);  // set log level
    // set log format
    // spdlog::set_pattern("%^[%Y%m%d %H:%M:%S.%e %L%P,%t%@]%$ %v");
    spdlog::set_pattern("%^[%Y%m%d %H:%M:%S.%e %L%P,%t%s%#]%$ %v");

    // create color multi threaded logger
    auto console = spdlog::stdout_color_mt("console");
    auto errLogger = spdlog::stdout_color_mt("stderr");
    spdlog::get("console")->info("logger can be retrieved from a global registry using the spdlog:get(logger_name)");
    spdlog::get("stderr")->info("another errlogger");

    auto fileLogger = spdlog::basic_logger_mt("basic_logger", "./logs/basicLog.txt");
    spdlog::get("basic_logger")->info("basic file logger");

    // set default logger with multi sink
    auto consoleSink = make_shared<spdlog::sinks::stderr_color_sink_mt>();
    consoleSink->set_level(spdlog::level::warn);  // console log only show warn and error message
    auto fileSink = make_shared<spdlog::sinks::basic_file_sink_mt>("logs/log.log", true);
    fileSink->set_level(spdlog::level::trace);  // file show all level
    // set it to default
    spdlog::set_default_logger(
        make_shared<spdlog::logger>("multi_sink", spdlog::sinks_init_list({consoleSink, fileSink})));
    spdlog::set_pattern("%^[%Y%m%d %H:%M:%S.%e %L%P,%t%s%#]%$ %v");

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