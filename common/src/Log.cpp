#include "common/Log.h"
#include <glog/logging.h>

namespace common {

// Initialize logging
void initLog(int argc, const char* argv[], int level, int verboseLevel) {
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    google::InstallFailureWriter([](const char* data, int size) { LOG(ERROR) << std::string(data, size); });
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;
    FLAGS_minloglevel = level;
    FLAGS_v = verboseLevel;
}

// Close log
void closeLog() { google::ShutdownGoogleLogging(); }

}  // namespace common