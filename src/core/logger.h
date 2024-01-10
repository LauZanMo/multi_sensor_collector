#pragma once

#include <Eigen/Core>
#include <glog/logging.h>

#define LOGI (LOG(INFO))
#define LOGW (LOG(WARNING))
#define LOGE (LOG(ERROR))
#define LOGF (LOG(FATAL))

namespace MSC {

class Logger {
public:
    static void initialize(char **argv, bool log_to_stderr, bool log_to_file);

    static std::string format(double data);

    static void shutdown();

    const static Eigen::IOFormat mat_fmt;
};

} // namespace MSC