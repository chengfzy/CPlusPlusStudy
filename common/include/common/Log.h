#pragma once
#include <glog/logging.h>

namespace common {

/**
 * @brief Initialize logging
 *
 * @param argc          Argument count
 * @param argv          Argument variable
 * @param level         Log level
 * @param verboseLevel  Verbose log level
 */
void initLog(int argc, const char* argv[], int level = google::INFO, int verboseLevel = 0);

/**
 * @brief Close log
 *
 */
void closeLog();

}  // namespace common
