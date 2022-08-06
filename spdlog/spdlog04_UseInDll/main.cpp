/**
 * @brief 在DLL中使用spdlog.
 *
 * 参考https://github.com/gabime/spdlog/wiki/How-to-use-spdlog-in-DLLs中的代码，感觉没啥意义，不会有任何差别.
 * 这里记录一下具体的使用方法
 *  1. setupLog()这句话必须要在DLL库中使用，才能保证后续的`spdlog::set_level()`生效.
 *  2. 在setupLog()中要调用`set_default_logger()`这句话, 官方写的`register_log()`没啥用, 也不需要在app中添加这句话
 *  3. `spdlog::set_level()`要注意和`SPDLOG_ACTIVE_LEVEL`的差别, `SPDLOG_ACTIVE_LEVEL`必须是编译命令,
 * 必须在使用之前添加, 否则没啥用. 而`spdlog::set_level()`是在`SPDLOG_ACTIVE_LEVEL`的基础上进行设置,
 * 默认`SPDLOG_ACTIVE_LEVEL`为Debug, 只有在`spdlog::set_level()`大于Debug时才有用
 *  4. 官方说的设置是针对为header_only, 但其实header_log就只需要调用DLL中的setupLog()即可, 其他不用管
 *  5. 如果使用的是spdlog的lib库，调用setupLog()或setupLogApp()均可
 *
 */

#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG
#include "MyLib.h"

#include <spdlog/async.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

using namespace std;
using namespace spdlog;

// Setup log
void setupLogApp() {
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

int main(int argc, const char* argv[]) {
    // setup log
    setupLogApp();
    // setupLog();
    spdlog::set_level(spdlog::level::trace);

    trace("trace log in app");
    debug("debug log in app");
    info("info log in app");
    error("error log in app");
    critical("critical log in app");

    SPDLOG_TRACE("trace log 2 in app");
    SPDLOG_DEBUG("debug log 2 in app");
    SPDLOG_INFO("info log 2 in app");
    SPDLOG_ERROR("error log 2 in app");
    SPDLOG_CRITICAL("critical log 2 in app");

    MyLib mylib;
    mylib.test();

    spdlog::info("Hello World!");
    spdlog::info("This is {:.3f}", 1.23546);

    spdlog::shutdown();
    return 0;
}