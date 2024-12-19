/*
 * Licensed under MIT License.
 * Author: Patrick Steil
*/

#ifndef STATUS_LOG_H
#define STATUS_LOG_H

#include <iostream>
#include <string>

#include "timer.h"

class StatusLog {
  StatusLog() = delete;
  StatusLog(const StatusLog &) = delete;
  StatusLog &operator=(const StatusLog &) = delete;
  long long time;

 public:
  StatusLog(const std::string &msg) {
    std::cout << msg << " ... " << std::flush;
    time = -get_micro_time();
  }
  StatusLog(const char *msg) {
    std::cout << msg << " ... " << std::flush;
    time = -get_micro_time();
  }
  ~StatusLog() {
    time += get_micro_time();
    std::cout << "done [" << time / 1000 << "ms]" << std::endl;
  }
};

#endif
