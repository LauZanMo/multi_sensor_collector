#include "core/logger.h"

#include <dvs_msgs/EventArray.h>
#include <highfive/H5Easy.hpp>
#include <rosbag/bag.h>
#include <sensor_msgs/Imu.h>

#include <absl/strings/str_cat.h>
#include <absl/strings/str_format.h>
#include <absl/strings/str_join.h>
#include <absl/strings/strip.h>
#include <absl/time/clock.h>
#include <fileio/file_base.h>
#include <fileio/file_writer.h>
#include <filesystem>
#include <iostream>
#include <tbb/parallel_for.h>
#include <yaml-cpp/yaml.h>

std::string msc_config_file = "../config/h52bag.yaml";

void writeEventMsg(rosbag::Bag &bag, const std::string &topic, const Eigen::MatrixXd &data, int width, int height);
void writeImuMsg(rosbag::Bag &bag, const std::string &topic, const Eigen::MatrixXd &data);
void writeImutxt(MSC::FileWriter::Ptr &file, const Eigen::MatrixXd &data);

struct SensorInfo {
    // hdf5信息
    std::string label;
    std::string path;
    size_t max_count;

    // bag信息
    std::string topic;
    std::string type;
    int width;
    int height;
};

int main(int argc, char **argv) {
    MSC::Logger::initialize(argv, true, true);

    YAML::Node msc_config;
    { // 加载配置文件
        try {
            msc_config = YAML::LoadFile(msc_config_file);
        } catch (YAML::Exception &exception) {
            std::cout << "Failed to open configuration file" << std::endl;
            return 1;
        }
    }

    auto log_path = msc_config["log_path"].as<std::string>();
    { // 配置输出
        if (!std::filesystem::is_directory(log_path))
            std::filesystem::create_directory(log_path);

        const absl::CivilSecond cs = absl::ToCivilSecond(absl::Now(), absl::LocalTimeZone());
        absl::StrAppendFormat(&log_path, "/T%04d%02d%02d%02d%02d%02d", cs.year(), cs.month(), cs.day(), cs.hour(),
                              cs.minute(), cs.second());
        std::filesystem::create_directory(log_path);

        FLAGS_log_dir = log_path;
    }

    // 搜索数据集
    std::vector<std::string> dataset_keys;
    std::vector<std::string> dataset_names;
    auto datasets_path = msc_config["datasets_path"].as<std::string>();
    for (const auto &entry : std::filesystem::directory_iterator(datasets_path)) {
        if (entry.is_directory()) {
            auto key = absl::StrCat(entry.path().string(), "/", entry.path().filename().string());
            dataset_keys.push_back(key);
            dataset_names.push_back(entry.path().filename().string());
        }
    }
    LOGI << "Found datasets: " << absl::StrJoin(dataset_names, ", ");

    tbb::parallel_for(tbb::blocked_range<size_t>(0, dataset_keys.size()), [&](const tbb::blocked_range<size_t> &r) {
        for (size_t i = r.begin(); i != r.end(); ++i) {
            auto &key  = dataset_keys[i];
            auto &name = dataset_names[i];

            auto h5_name = absl::StrCat(key, ".h5");
            H5Easy::File h5(h5_name, H5Easy::File::ReadOnly);

            auto bag_name = absl::StrCat(key, ".bag");
            rosbag::Bag bag(bag_name, rosbag::bagmode::Write);
            bag.setChunkThreshold(1024 * 1024);

            // 读取hdf5配置信息
            auto bag_config     = msc_config["bag_writer"];
            auto sensors_config = bag_config["sensors"];
            std::vector<SensorInfo> sensor_infos;
            for (const auto &sensor_config : sensors_config) {
                auto label     = sensor_config["label"].as<std::string>();
                auto path      = "/" + label + "/";
                auto max_count = h5.getGroup(label).getNumberObjects();

                auto topic = sensor_config["topic"].as<std::string>();
                auto type  = sensor_config["type"].as<std::string>();
                int width = 0, height = 0;
                if (type == "event") {
                    width  = sensor_config["width"].as<int>();
                    height = sensor_config["height"].as<int>();
                }
                sensor_infos.push_back({label, path, max_count, topic, type, width, height});
            }

            MSC::FileWriter::Ptr imu_txt{nullptr};
            for (auto &info : sensor_infos) {
                // 如果时传感器时imu，需要额外输出txt文件
                if (info.type == "imu") {
                    auto file_name = absl::StrCat(key, "_", info.label, ".txt");
                    imu_txt        = MSC::FileWriter::create(file_name);
                }

                double ratio = 0.1;
                for (size_t j = 0; j < info.max_count; ++j) {
                    auto path = info.path + std::to_string(j);
                    if (info.type == "event") {
                        auto data = H5Easy::load<Eigen::MatrixXd>(h5, path);
                        writeEventMsg(bag, info.topic, data, info.width, info.height);
                    } else if (info.type == "imu") {
                        auto data = H5Easy::load<Eigen::MatrixXd>(h5, path);
                        writeImuMsg(bag, info.topic, data);
                        writeImutxt(imu_txt, data);
                    }

                    if (j > info.max_count * ratio) {
                        LOGI << "Dataset " << name << ": " << info.label
                             << absl::StrFormat(" progress to: %.2f%%", ratio * 100);
                        ratio += 0.1;
                    }
                }
                LOGI << "Dataset " << name << ": " << info.label << absl::StrFormat(" progress to: %.2f%%", 100.0);
            }

            bag.close();
            LOGI << "Dataset " << name << " finished.";
        }
    });

    MSC::Logger::shutdown();
    return 0;
}

void writeEventMsg(rosbag::Bag &bag, const std::string &topic, const Eigen::MatrixXd &data, int width, int height) {
    dvs_msgs::EventArray msg;
    msg.header.stamp.fromSec(data(data.rows() - 1, 0));
    msg.width  = width;
    msg.height = height;
    msg.events.reserve(data.rows());
    for (int i = 0; i < data.rows(); ++i) {
        dvs_msgs::Event event;
        event.ts.fromSec(data(i, 0));
        event.x        = std::round(data(i, 1));
        event.y        = std::round(data(i, 2));
        event.polarity = data(i, 3) > 0.5 ? true : false;
        msg.events.push_back(event);
    }
    bag.write(topic, msg.header.stamp, msg);
}

void writeImuMsg(rosbag::Bag &bag, const std::string &topic, const Eigen::MatrixXd &data) {
    sensor_msgs::Imu msg;
    msg.header.stamp.fromSec(data(0, 0));
    msg.angular_velocity.x    = data(0, 1);
    msg.angular_velocity.y    = data(0, 2);
    msg.angular_velocity.z    = data(0, 3);
    msg.linear_acceleration.x = data(0, 4);
    msg.linear_acceleration.y = data(0, 5);
    msg.linear_acceleration.z = data(0, 6);
    bag.write(topic, msg.header.stamp, msg);
}

void writeImutxt(MSC::FileWriter::Ptr &file, const Eigen::MatrixXd &data) {
    std::vector<double> imu;
    for (uint8_t i = 0; i < 7; ++i) {
        imu.push_back(data(0, i));
    }
    file->dump(imu);
}