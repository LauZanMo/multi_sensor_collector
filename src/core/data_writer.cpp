#include "core/data_writer.h"
#include "core/synchronizer.h"

namespace MSC {

DataWriter::DataWriter(const YAML::Node &config) {
    auto data_writer_config = config["data_writer"];

    file_name_ = data_writer_config["file_name"].as<std::string>();
}

void DataWriter::init() {
    file_    = std::make_shared<H5Easy::File>(file_name_,
                                           H5Easy::File::ReadWrite | H5Easy::File::Create | H5Easy::File::Truncate);
    options_ = H5Easy::DumpOptions(H5Easy::Compression(1));

    auto sensors = synchronizer_->sensors();
    for (auto &sensor_pair : sensors) {
        auto &label         = sensor_pair.first;
        dataset_idx_[label] = 0;
    }
}

void DataWriter::process() {
    LabeledDataPtr ld;
    while (synchronizer_->isRunning() || !data_queue_.empty()) {
        if (data_queue_.try_pop(ld)) {
            auto &label = ld->label;
            auto path   = absl::StrCat("/", label, "/", dataset_idx_[label]++);
            H5Easy::dump(*file_, path, ld->data, options_);
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

} // namespace MSC