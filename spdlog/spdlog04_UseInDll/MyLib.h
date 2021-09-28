#pragma once
#include <spdlog/spdlog.h>
#include <memory>

/**
 * @brief Setup log
 *
 * @return  The logger pointer
 */
void setupLog();

class MyLib {
  public:
    void test();

  private:
};