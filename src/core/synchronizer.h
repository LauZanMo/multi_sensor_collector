#pragma once

#include "core/sensor_base.h"
#include "core/serial_helper.h"
#include "core/ui.h"
#include "data_writer.h"

#include <thread>
#include <unordered_map>
#include <yaml-cpp/yaml.h>

namespace MSC {

class Synchronizer {
public:
    typedef std::shared_ptr<Synchronizer> Ptr;

    explicit Synchronizer(const YAML::Node &config);
    ~Synchronizer();

    void init();
    void process();

    bool isRunning() const {
        return is_running_;
    }

    void shutdown() {
        is_running_ = false;
    }

    void setUI(const UI::Ptr &ui) {
        ui_ = ui;
    }

    void setDataWriter(const DataWriter::Ptr &data_writer) {
        data_writer_ = data_writer;
    }

    DataWriter::Ptr dataWriter() const {
        return data_writer_;
    }

    std::unordered_map<std::string, SensorBase::Ptr> sensors() const {
        return sensors_;
    }

private:
    void processExtTrigger();

    void parse(const std::string &data);

    SerialParam serial_param_;
    serial::Serial serial_;

    UI::Ptr ui_;
    DataWriter::Ptr data_writer_;

    // 容器
    std::unordered_map<std::string, SensorBase::Ptr> sensors_;
    std::vector<std::thread> threads_;

    // 标志位
    std::atomic<bool> is_running_{false};

    // 属性
    std::string label_;
};

} // namespace MSC