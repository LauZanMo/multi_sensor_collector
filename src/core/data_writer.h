#pragma once

#include "core/types.h"

#include <highfive/H5Easy.hpp>
#include <memory>
#include <tbb/concurrent_queue.h>
#include <unordered_map>
#include <yaml-cpp/yaml.h>

namespace MSC {

class Synchronizer;
typedef std::shared_ptr<Synchronizer> SynchronizerPtr;

class DataWriter {
public:
    typedef std::shared_ptr<DataWriter> Ptr;

    explicit DataWriter(const YAML::Node &config);
    ~DataWriter() = default;

    void init();
    void process();

    void write(const LabeledDataPtr &data) {
        data_queue_.push(data);
    }

    void setSynchronizer(const SynchronizerPtr &synchronizer) {
        synchronizer_ = synchronizer;
    }

private:
    std::shared_ptr<H5Easy::File> file_;
    H5Easy::DumpOptions options_;
    std::unordered_map<std::string, long> dataset_idx_;

    SynchronizerPtr synchronizer_;

    tbb::concurrent_queue<LabeledDataPtr> data_queue_;

    // 参数
    std::string file_name_;
};

} // namespace MSC