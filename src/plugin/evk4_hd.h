#pragma once

#include <metavision/sdk/core/algorithms/periodic_frame_generation_algorithm.h>
#include <metavision/sdk/driver/camera.h>
#include <tbb/concurrent_queue.h>
#include <thread>
#include <yaml-cpp/yaml.h>

#include "core/sensor_base.h"

namespace MSC {

class Evk4Hd : public SensorBase {
public:
    Evk4Hd(const YAML::Node &config, const YAML::Node &sensor_config);
    virtual ~Evk4Hd() override;

    void init() override;
    void process() override;

    void insertExtTrigger(const StampBundle &stamp) override;

    void visualizeStart() override;
    void visualizeStop() override;

    void dump(HighFive::FilePtr &file, const std::string &path, const LabeledDataBasePtr &data) override;
    static EventsDataPtr load(HighFive::FilePtr &file, const std::string &path);

private:
    bool openCamera();
    void processCD(const Metavision::EventCD *begin, const Metavision::EventCD *end);
    void processExtTrigger(const Metavision::EventExtTrigger *begin, const Metavision::EventExtTrigger *end);

    Metavision::Camera camera_;
    std::shared_ptr<Metavision::PeriodicFrameGenerationAlgorithm> frame_gen_;

    tbb::concurrent_queue<cv::Mat> frame_queue_;
    std::atomic<bool> visualize_{false};
    std::shared_ptr<std::thread> viz_thread_;

    // 容器
    tbb::concurrent_queue<StampBundle> sensor_stamp_queue_, ext_stamp_queue_;
    EventsDataPtr ld_;
    long current_row_;
    std::atomic<bool> offset_set_{false}, last_offset_set_{false}, offset_reset_{false};
    std::atomic<double> time_offset_{0.0};

    // 参数
    std::string bias_file_;
    uint32_t rate_;
    double sync_threshold_;
    long max_size_;
    uint32_t acc_size_;
};

} // namespace MSC