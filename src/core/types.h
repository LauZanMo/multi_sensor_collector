#pragma once

#include <Eigen/Core>
#include <absl/time/clock.h>
#include <memory>

namespace MSC {

struct StampBundle {
    absl::Time stamp_local;
    double stamp_trigger;
};

struct LabeledDataBase {
    std::string label;
    Eigen::Matrix<double, Eigen::Dynamic, 1> stamps;
};
typedef std::shared_ptr<LabeledDataBase> LabeledDataBasePtr;

struct EventsData : public LabeledDataBase {
    Eigen::Matrix<uint16_t, Eigen::Dynamic, 2> locs;
    Eigen::Matrix<uint8_t, Eigen::Dynamic, 1> pols;
};
typedef std::shared_ptr<EventsData> EventsDataPtr;

struct ImuData : public LabeledDataBase {
    Eigen::Matrix<double, Eigen::Dynamic, 6> data;
};
typedef std::shared_ptr<ImuData> ImuDataPtr;

} // namespace MSC