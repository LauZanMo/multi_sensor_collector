#include "core/logger.h"
#include "core/types.h"
#include "plugin/evk4_hd.h"
#include "plugin/ins_probe.h"

#include <dvs_msgs/EventArray.h>
#include <highfive/H5Easy.hpp>
#include <rosbag/bag.h>
#include <sensor_msgs/Imu.h>

#include <absl/strings/str_cat.h>
#include <absl/strings/str_format.h>
#include <absl/strings/str_join.h>
#include <absl/strings/strip.h>
#include <absl/time/clock.h>

#include <fileio/file_writer.h>
#include <filesystem>
#include <iostream>
#include <tbb/parallel_for.h>
#include <yaml-cpp/yaml.h>

std::string msc_config_file = "../config/h52bag.yaml";

void writeEventMsg(rosbag::Bag &bag, const std::string &topic, const std::vector<std::string> &params,
                   const MSC::EventsDataPtr &data);
void writeImuMsg(rosbag::Bag &bag, const std::string &topic, const std::vector<std::string> &params,
                 const MSC::ImuDataPtr &data);
void writeImutxt(MSC::FileWriter::Ptr &file, const std::vector<std::string> &params, const MSC::ImuDataPtr &data);

struct SensorInfo {
    // hdf5信息
    std::string label;
    std::string path;
    size_t max_count;

    // bag信息
    std::string topic;
    std::string type;
    std::vector<std::string> params;
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
            auto name = entry.path().filename().string();
            if (name == "output" || name[0] == '.')
                continue;
            auto key = absl::StrCat(entry.path().string(), "/", name);
            dataset_keys.push_back(key);
            dataset_names.push_back(name);
        }
    }
    LOGI << "Found datasets: " << absl::StrJoin(dataset_names, ", ");

    tbb::parallel_for(tbb::blocked_range<size_t>(0, dataset_keys.size()), [&](const tbb::blocked_range<size_t> &r) {
        for (size_t i = r.begin(); i != r.end(); ++i) {
            auto &key  = dataset_keys[i];
            auto &name = dataset_names[i];

            auto h5_name = absl::StrCat(key, ".h5");
            auto h5      = std::make_shared<H5Easy::File>(h5_name, H5Easy::File::ReadOnly);

            auto bag_name = absl::StrCat(key, ".bag");
            rosbag::Bag bag(bag_name, rosbag::bagmode::Write);
            bag.setChunkThreshold(1024 * 1024);

            // 读取hdf5配置信息
            auto bag_config     = msc_config["bag_writer"];
            auto sensors_config = bag_config["sensors"];
            std::vector<SensorInfo> sensor_infos;
            for (const auto &sensor_config : sensors_config) {
                auto label     = sensor_config["label"].as<std::string>();
                auto path      = absl::StrCat("/", label, "/");
                auto max_count = h5->getGroup(label).getNumberObjects();

                auto topic  = sensor_config["topic"].as<std::string>();
                auto type   = sensor_config["type"].as<std::string>();
                auto params = sensor_config["params"].as<std::vector<std::string>>();
                sensor_infos.push_back({label, path, max_count, topic, type, params});
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
                    auto path = absl::StrCat(info.path, j);
                    if (info.type == "event") {
                        auto data = MSC::Evk4Hd::load(h5, path);
                        writeEventMsg(bag, info.topic, info.params, data);
                    } else if (info.type == "imu") {
                        auto data = MSC::InsProbe::load(h5, path);
                        writeImuMsg(bag, info.topic, info.params, data);
                        writeImutxt(imu_txt, info.params, data);
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

void writeEventMsg(rosbag::Bag &bag, const std::string &topic, const std::vector<std::string> &params,
                   const MSC::EventsDataPtr &data) {
    dvs_msgs::EventArray msg;
    absl::SimpleAtoi(params[0], &msg.width);
    absl::SimpleAtoi(params[1], &msg.height);
    double rate;
    absl::SimpleAtod(params[2], &rate);
    auto duration = 1.0 / rate;

    long start_row = 0;
    dvs_msgs::Event event;
    for (auto i = 0; i < data->stamps.rows(); ++i) {
        event.ts.fromSec(data->stamps(i, 0));
        event.x        = data->locs(i, 0);
        event.y        = data->locs(i, 1);
        event.polarity = data->pols(i, 0);
        msg.events.push_back(event);

        if (data->stamps(i, 0) - data->stamps(start_row, 0) >= duration || i == data->stamps.rows() - 1) {
            msg.header.stamp.fromSec(data->stamps(i, 0));
            bag.write(topic, msg.header.stamp, msg);
            msg.events.clear();
            start_row = i;
        }
    }
}

void writeImuMsg(rosbag::Bag &bag, const std::string &topic, const std::vector<std::string> &params,
                 const MSC::ImuDataPtr &data) {
    sensor_msgs::Imu msg;
    for (auto i = 0; i < data->stamps.rows(); ++i) {
        msg.header.stamp.fromSec(data->stamps(i, 0));
        msg.angular_velocity.x    = data->data(i, 0);
        msg.angular_velocity.y    = data->data(i, 1);
        msg.angular_velocity.z    = data->data(i, 2);
        msg.linear_acceleration.x = data->data(i, 3);
        msg.linear_acceleration.y = data->data(i, 4);
        msg.linear_acceleration.z = data->data(i, 5);
        bag.write(topic, msg.header.stamp, msg);
    }
}

void writeImutxt(MSC::FileWriter::Ptr &file, const std::vector<std::string> &params, const MSC::ImuDataPtr &data) {
    std::vector<double> imu;
    for (auto i = 0; i < data->stamps.rows(); ++i) {
        imu.clear();
        imu.push_back(data->stamps(i, 0));
        for (uint8_t j = 0; j < 6; ++j) {
            imu.push_back(data->data(i, j));
        }
        file->dump(imu);
    }
}