#include "core/logger.h"

#include <absl/strings/str_format.h>

namespace MSC {

void Logger::initialize(char **argv, bool log_to_stderr, bool log_to_file) {
    google::InitGoogleLogging(argv[0]);

    FLAGS_logtostderr = log_to_stderr;
    if (log_to_stderr)
        FLAGS_colorlogtostderr = true;
    if (log_to_stderr && log_to_file)
        FLAGS_alsologtostderr = true;
}

std::string Logger::format(double data) {
    return absl::StrFormat("%0.6lf", data);
}

void Logger::shutdown() {
    google::ShutdownGoogleLogging();
}

const Eigen::IOFormat Logger::mat_fmt{15, 0, ", ", ",\n", "", "", "\n[", "]"};

} // namespace MSC