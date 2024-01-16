#include "plugin/ins_probe.h"

#include <core/logger.h>
#include <core/synchronizer.h>
#include <highfive/H5Easy.hpp>

namespace MSC {

InsProbe::InsProbe(const YAML::Node &config, const YAML::Node &sensor_config)
    : SensorBase() {
    type_           = Type::IMU;
    require_thread_ = true;

    label_                    = sensor_config["label"].as<std::string>();
    rate_                     = sensor_config["rate"].as<uint32_t>();
    serial_param_.port        = sensor_config["port"].as<std::string>();
    serial_param_.baudrate    = sensor_config["baudrate"].as<uint32_t>();
    const auto timeout        = sensor_config["timeout"].as<uint32_t>();
    serial_param_.timeout     = serial::Timeout::simpleTimeout(timeout);
    serial_param_.bytesize    = serial::bytesize_t::eightbits;
    serial_param_.parity      = serial::parity_t::parity_none;
    serial_param_.stopbits    = serial::stopbits_t::stopbits_one;
    serial_param_.flowcontrol = serial::flowcontrol_t::flowcontrol_none;
}

void InsProbe::init() {
    openSerial(serial_, serial_param_, label_);
}

void InsProbe::process() {
    double prev_t         = 0;
    double last_dt_inv    = 200.0;
    bool is_imu_published = false;
    uint8_t data;

    while (synchronizer_->isRunning()) {
        if (serial_.waitReadable()) {
            // 解析
            serial_.read(&data, 1);
            if (parse(data)) {
                if (prev_t == 0) { // 第一帧
                    prev_t = insprobe_imu_.week_second;
                } else {
                    // 检查丢数
                    double dt = insprobe_imu_.week_second - prev_t;
                    double dt_inv;
                    if (dt > 0.008 || dt < 0.003) {
                        dt_inv = last_dt_inv;
                        LOGW << label_ << ": IMU data lost!";
                    } else {
                        dt_inv      = 1.0 / dt;
                        last_dt_inv = dt_inv;
                    }
                    prev_t = insprobe_imu_.week_second;

                    if (!ld_) {
                        ld_        = std::make_shared<ImuData>();
                        ld_->label = label_;
                        ld_->stamps.resize(rate_, 1);
                        ld_->data.resize(rate_, 6);
                        current_row_ = 0;
                    }

                    Eigen::Matrix<double, 1, 6> imu_inc;

                    // 转换为写入格式：增量式和绝对式
                    ld_->stamps(current_row_, 0) = insprobe_imu_.week_second;

                    imu_inc(0, 0) = insprobe_imu_.gyro_x * DEG2RAD;
                    imu_inc(0, 1) = insprobe_imu_.gyro_y * DEG2RAD;
                    imu_inc(0, 2) = insprobe_imu_.gyro_z * DEG2RAD;
                    imu_inc(0, 3) = insprobe_imu_.acc_x;
                    imu_inc(0, 4) = insprobe_imu_.acc_y;
                    imu_inc(0, 5) = insprobe_imu_.acc_z;

                    ld_->data.block<1, 6>(current_row_, 0) = imu_inc * dt_inv;
                    current_row_++;

                    // 写入
                    if (current_row_ == rate_) {
                        if (record_) {
                            auto writer = synchronizer_->dataWriter();
                            writer->write(std::static_pointer_cast<LabeledDataBase>(ld_));
                        }

                        ld_.reset();
                    }

                    // 可视化
                    if (visualize_) {
                        // LOGI << label_ << ": IMU data Inc: " << imu_inc.transpose().format(Logger::mat_fmt);
                    }

                    if (!is_imu_published) {
                        LOGI << label_ << ": IMU data published.";
                        is_imu_published = true;
                    }
                }
            }
        }
    }
    serial_.close();
}

void InsProbe::insertExtTrigger(const StampBundle &ext) {
    LOGW << label_ << ": INS-Probe does not support external trigger.";
}

void InsProbe::visualizeStart() {
    LOGI << label_ << ": IMU data visualization started.";
    visualize_ = true;
}

void InsProbe::visualizeStop() {
    LOGI << label_ << ": IMU data visualization stopped.";
    visualize_ = false;
}

void InsProbe::dump(HighFive::FilePtr &file, const std::string &path, const LabeledDataBasePtr &data) {
    auto imu_data = std::static_pointer_cast<ImuData>(data);
    H5Easy::dump(*file, absl::StrCat(path, "/stamp"), imu_data->stamps);
    H5Easy::dump(*file, absl::StrCat(path, "/data"), imu_data->data);
}

ImuDataPtr InsProbe::load(HighFive::FilePtr &file, const std::string &path) {
    auto imu_data    = std::make_shared<ImuData>();
    imu_data->stamps = H5Easy::load<Eigen::Matrix<double, Eigen::Dynamic, 1>>(*file, absl::StrCat(path, "/stamp"));
    imu_data->data   = H5Easy::load<Eigen::Matrix<double, Eigen::Dynamic, 6>>(*file, absl::StrCat(path, "/data"));
    return imu_data;
}

bool InsProbe::parse(const uint8_t &data) {
    if (counter_ == 0)
        checksum_ = 0;

    if (counter_ < 2) { // 接收包头
        if (data != packet_ID[counter_])
            counter_ = 0;
        else
            counter_++;
    } else if (counter_ < 44) { // 接收数据
        ((uint8_t *) &insprobe_imu_)[counter_ - 2] = data;
        checksum_ += data;
        counter_++;
    } else if (counter_ < 45) { // 接收校验和（这里我只用了第一个校验位）
        counter_++;
        if (checksum_ == data)
            return true;
    } else { // 忽略第二个校验位
        counter_ = 0;
    }
    return false;
}

} // namespace MSC