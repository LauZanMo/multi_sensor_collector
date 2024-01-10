#pragma once

#include <Eigen/Core>
#include <absl/time/clock.h>
#include <memory>

namespace MSC {

struct StampBundle {
    absl::Time stamp_local;
    double stamp_trigger;
};

struct LabeledData {
    std::string label;
    Eigen::MatrixXd data;
};
typedef std::shared_ptr<LabeledData> LabeledDataPtr;

} // namespace MSC