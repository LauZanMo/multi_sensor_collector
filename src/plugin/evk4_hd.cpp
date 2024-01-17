#include "plugin/evk4_hd.h"
#include "core/logger.h"

#include <core/synchronizer.h>
#include <highfive/H5Easy.hpp>
#include <metavision/hal/facilities/i_erc.h>
#include <metavision/hal/facilities/i_trigger_in.h>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <tbb/parallel_for.h>
#include <thread>

namespace MSC {

Evk4Hd::Evk4Hd(const YAML::Node &config, const YAML::Node &sensor_config)
    : SensorBase() {
    type_           = Type::EVENT;
    require_thread_ = false;

    label_          = sensor_config["label"].as<std::string>();
    bias_file_      = sensor_config["bias_file"].as<std::string>();
    rate_           = sensor_config["rate"].as<uint32_t>();
    max_size_       = rate_;
    auto sync_rate  = sensor_config["sync_rate"].as<double>();
    sync_threshold_ = 0.1 / sync_rate;
    acc_size_       = sensor_config["acc_size"].as<uint32_t>();
}

Evk4Hd::~Evk4Hd() {
    if (camera_.is_running())
        camera_.stop();
}

void Evk4Hd::init() {
    // 打开相机
    while (!openCamera()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        LOGI << label_ << ": Trying to open camera...";
        if (!synchronizer_->isRunning())
            return;
    }

    camera_.add_runtime_error_callback(
        [this](const Metavision::CameraException &e) { LOGE << label_ << ": " << e.what(); });

    // 使能外部触发
    camera_.get_device().get_facility<Metavision::I_TriggerIn>()->enable(0);

    // 事件流数据量控制
    camera_.get_device().get_facility<Metavision::I_Erc>()->set_cd_event_rate(rate_);
    camera_.get_device().get_facility<Metavision::I_Erc>()->enable(true);

    // 打印相机信息
    auto &config   = camera_.get_camera_configuration();
    auto &geometry = camera_.geometry();
    auto width = geometry.width(), height = geometry.height();
    LOGI << label_ << ": Camera geometry " << width << "x" << height;
    LOGI << label_ << ": Camera serial number: " << config.serial_number;

    // 加入UI
    auto fps   = 50.0;
    frame_gen_ = std::make_shared<Metavision::PeriodicFrameGenerationAlgorithm>(width, height, acc_size_, fps);
    cv::namedWindow(label_, cv::WINDOW_NORMAL);
    cv::resizeWindow(label_, width, height);
    frame_gen_->set_output_callback([this](Metavision::timestamp, cv::Mat &frame) {
        if (visualize_)
            frame_queue_.push(frame);
    });
}

void Evk4Hd::process() {
    camera_.start();

    camera_.cd().add_callback(
        [this](const Metavision::EventCD *begin, const Metavision::EventCD *end) { processCD(begin, end); });

    camera_.cd().add_callback([&](const Metavision::EventCD *begin, const Metavision::EventCD *end) {
        frame_gen_->process_events(begin, end);
    });

    camera_.ext_trigger().add_callback(
        [this](const Metavision::EventExtTrigger *begin, const Metavision::EventExtTrigger *end) {
            processExtTrigger(begin, end);
        });
}

void Evk4Hd::insertExtTrigger(const StampBundle &ext) {
    ext_stamp_queue_.push(ext);
}

void Evk4Hd::visualizeStart() {
    visualize_ = true;
    if (!viz_thread_) {
        viz_thread_ = std::make_shared<std::thread>([this]() {
            while (visualize_) {
                cv::Mat frame;
                if (frame_queue_.try_pop(frame)) {
                    cv::imshow(label_, frame);
                    cv::waitKey(1);
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                }
            }
        });
    }
}

void Evk4Hd::visualizeStop() {
    visualize_ = false;
    if (viz_thread_) {
        if (viz_thread_->joinable())
            viz_thread_->join();
        viz_thread_.reset();
    }
}

void Evk4Hd::dump(HighFive::FilePtr &file, const std::string &path, const LabeledDataBasePtr &data) {
    auto events_data = std::static_pointer_cast<EventsData>(data);
    H5Easy::dump(*file, absl::StrCat(path, "/stamp"), events_data->stamps);
    H5Easy::dump(*file, absl::StrCat(path, "/loc"), events_data->locs);
    H5Easy::dump(*file, absl::StrCat(path, "/pol"), events_data->pols);
}

EventsDataPtr Evk4Hd::load(HighFive::FilePtr &file, const std::string &path) {
    auto events_data    = std::make_shared<EventsData>();
    events_data->stamps = H5Easy::load<Eigen::Matrix<double, Eigen::Dynamic, 1>>(*file, absl::StrCat(path, "/stamp"));
    events_data->locs   = H5Easy::load<Eigen::Matrix<uint16_t, Eigen::Dynamic, 2>>(*file, absl::StrCat(path, "/loc"));
    events_data->pols   = H5Easy::load<Eigen::Matrix<uint8_t, Eigen::Dynamic, 1>>(*file, absl::StrCat(path, "/pol"));
    return events_data;
}

bool Evk4Hd::openCamera() {
    try {
        camera_ = Metavision::Camera::from_first_available();

        if (!bias_file_.empty()) {
            LOGI << label_ << ": Loading bias file: " << bias_file_;
            camera_.biases().set_from_file(bias_file_);
        }

        return true;
    } catch (Metavision::CameraException &e) {
        LOGE << label_ << ": " << e.what();
    }
    return false;
}

void Evk4Hd::processCD(const Metavision::EventCD *begin, const Metavision::EventCD *end) {
    if (end > begin) {
        const auto size = end - begin;
        if (!ld_) {
            ld_        = std::make_shared<EventsData>();
            ld_->label = label_;
            ld_->stamps.resize(max_size_);
            ld_->locs.resize(max_size_, 2);
            ld_->pols.resize(max_size_);
            current_row_ = 0;
        }

        for (long i = 0; i < size; i++, current_row_++) {
            if (current_row_ == max_size_)
                break;
            auto &e                      = begin[i];
            ld_->stamps(current_row_, 0) = e.t * 1e-6 - time_offset_;
            ld_->locs(current_row_, 0)   = e.x;
            ld_->locs(current_row_, 1)   = e.y;
            ld_->pols(current_row_, 0)   = e.p;
        }

        auto start_t = ld_->stamps(0, 0);
        auto end_t   = ld_->stamps(current_row_ - 1, 0);
        if (end_t - start_t > 1.0 || current_row_ == max_size_) {
            if (current_row_ != max_size_) {
                ld_->stamps.conservativeResize(current_row_, Eigen::NoChange);
                ld_->locs.conservativeResize(current_row_, Eigen::NoChange);
                ld_->pols.conservativeResize(current_row_, Eigen::NoChange);
            }

            if (record_ && offset_set_) {
                if (last_offset_set_) {
                    auto writer = synchronizer_->dataWriter();
                    writer->write(std::static_pointer_cast<LabeledDataBase>(ld_));
                } else {
                    last_offset_set_ = true;
                }
            }

            ld_.reset();
        }
    }
}

void Evk4Hd::processExtTrigger(const Metavision::EventExtTrigger *begin, const Metavision::EventExtTrigger *end) {
    if (end > begin) {
        for (auto e = begin; e != end; e++) {
            StampBundle stamp_bundle;
            stamp_bundle.stamp_local   = absl::Now();
            stamp_bundle.stamp_trigger = e->t * 1e-6;
            sensor_stamp_queue_.emplace(stamp_bundle);
        }

        StampBundle stamp_bundle_sensor, stamp_bundle_ext;
        while (sensor_stamp_queue_.try_pop(stamp_bundle_sensor)) {
            auto local_sensor = stamp_bundle_sensor.stamp_local;

            while (ext_stamp_queue_.try_pop(stamp_bundle_ext)) {
                auto local_ext = stamp_bundle_ext.stamp_local;
                auto diff      = absl::ToDoubleSeconds(local_sensor - local_ext);
                auto abs_diff  = std::abs(diff);

                if (diff < 0.0 && abs_diff > sync_threshold_)
                    break;

                if (abs_diff < sync_threshold_) {
                    auto offset = stamp_bundle_sensor.stamp_trigger - stamp_bundle_ext.stamp_trigger;
                    if (offset_set_) {
                        if (std::abs(offset - time_offset_) < 1e-4) {
                            time_offset_ = offset;
                        } else {
                            LOGW << label_ << ": Time offset is not stable!";
                        }
                    } else {
                        time_offset_ = offset;
                        LOGI << label_ << ": Time offset is initialized to " << time_offset_ << "s.";
                        offset_set_ = true;
                    }
                    break;
                }
            }
        }
    }
}


} // namespace MSC