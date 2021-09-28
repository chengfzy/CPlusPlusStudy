#include "MyLib.h"
#include <spdlog/async.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

using namespace std;
using namespace spdlog;

// Setup log
void setupLog() {
    auto myLogger = get("async");
    if (!myLogger) {
        // console log
        auto consoleSink = std::make_shared<sinks::stderr_color_sink_mt>();
        consoleSink->set_level(level::trace);

        // file sink show all level logs, and rotate log with max 50Mb size and 10 rotated files
        auto fileSink = std::make_shared<sinks::rotating_file_sink_mt>("./logs/log.log", 1024 * 1024 * 50, 10);
        fileSink->set_level(level::trace);

        // create async logger with console and file sync
        init_thread_pool(8192, 1);  // queue with 8k items and 1 backing thread
        myLogger = std::make_shared<async_logger>("async", sinks_init_list({consoleSink, fileSink}), thread_pool(),
                                                  async_overflow_policy::overrun_oldest);
        set_default_logger(myLogger);

        // set level and pattern
        set_pattern("%^[%Y%m%d %H:%M:%S.%e %L%P,%t %s:%#]%$ %v");  // set global pattern
    }
}

void MyLib::test() {
    trace("trace log in lib");
    debug("debug log in lib");
    info("info log in lib");
    error("error log in lib");
    critical("critical log in lib");

    SPDLOG_TRACE("trace log 2 in lib");
    SPDLOG_DEBUG("debug log 2 in lib");
    SPDLOG_INFO("info log 2 in lib");
    SPDLOG_ERROR("error log 2 in lib");
    SPDLOG_CRITICAL("critical log 2 in lib");
}