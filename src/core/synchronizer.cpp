#include "core/synchronizer.h"
#include "plugin/evk4_hd.h"
#include "plugin/ins_probe.h"

#include <absl/strings/numbers.h>
#include <absl/strings/str_split.h>
#include <core/logger.h>
#include <yaml-cpp/yaml.h>

namespace MSC {

Synchronizer::Synchronizer(const YAML::Node &config) {
    auto synchronizer_config = config["synchronizer"];
    auto sensors_config      = config["sensors"];

    label_                    = synchronizer_config["label"].as<std::string>();
    serial_param_.port        = synchronizer_config["port"].as<std::string>();
    serial_param_.baudrate    = synchronizer_config["baudrate"].as<uint32_t>();
    const auto &timeout       = synchronizer_config["timeout"].as<uint32_t>();
    serial_param_.timeout     = serial::Timeout::simpleTimeout(timeout);
    serial_param_.bytesize    = serial::bytesize_t::eightbits;
    serial_param_.parity      = serial::parity_t::parity_none;
    serial_param_.stopbits    = serial::stopbits_t::stopbits_one;
    serial_param_.flowcontrol = serial::flowcontrol_t::flowcontrol_none;

    for (const auto &sensor_config : sensors_config) {
        auto model = sensor_config["model"].as<std::string>();
        if (model == "INS-Probe") {
            sensors_[sensor_config["label"].as<std::string>()] = std::make_shared<InsProbe>(config, sensor_config);
        } else if (model == "EVK4-HD") {
            sensors_[sensor_config["label"].as<std::string>()] = std::make_shared<Evk4Hd>(config, sensor_config);
        } else {
            LOGE << label_ << ": Unknown sensor type: " << model;
        }
    }
}

Synchronizer::~Synchronizer() {
    for (auto &sensor_pair : sensors_) {
        auto &sensor = sensor_pair.second;
        sensor->recordStop();
    }

    if (is_running_)
        shutdown();

    for (auto &thread : threads_) {
        if (thread.joinable())
            thread.join();
    }

    sensors_.clear();
}

void Synchronizer::init() {
    openSerial(serial_, serial_param_, label_);
    is_running_ = true;

    for (auto &sensor_pair : sensors_) {
        auto &label  = sensor_pair.first;
        auto &sensor = sensor_pair.second;
        sensor->setSynchronizer(Synchronizer::Ptr(this));
        sensor->init();
        LOGI << label_ << ": Sensor " << label << " initialized.";

        sensor->recordStart();
    }

    data_writer_->init();
}

void Synchronizer::process() {
    threads_.emplace_back(&Synchronizer::processExtTrigger, this);
    threads_.emplace_back(&DataWriter::process, data_writer_);

    for (auto &sensor_pair : sensors_) {
        auto &sensor = sensor_pair.second;
        if (sensor->requireThread())
            threads_.emplace_back(&SensorBase::process, sensor);
        else
            sensor->process();
    }

    processExtTrigger();
}

void Synchronizer::processExtTrigger() {
    while (isRunning()) {
        if (serial_.waitReadable()) {
            auto data = serial_.readline();
            parse(data);
        }
    }
    serial_.close();
}

void Synchronizer::parse(const std::string &data) {
    std::vector<absl::string_view> v = absl::StrSplit(data, absl::ByAnyChar("$,"));

    StampBundle stamp_bundle;
    stamp_bundle.stamp_local = absl::Now();
    absl::SimpleAtod(v[3], &stamp_bundle.stamp_trigger);

    auto &sensor = sensors_[std::string(v[1])];
    sensor->insertExtTrigger(stamp_bundle);
}

} // namespace MSC