#pragma once

#include <atomic>
#include <memory>

#include "core/types.h"

namespace MSC {

class Synchronizer;
typedef std::shared_ptr<Synchronizer> SynchronizerPtr;

class SensorBase {
public:
    typedef std::shared_ptr<SensorBase> Ptr;

    enum class Type { INVALID = -1, IMU = 0, EVENT = 1, CAMERA = 2 };

    SensorBase()          = default;
    virtual ~SensorBase() = default;

    virtual void init()    = 0;
    virtual void process() = 0;

    virtual void insertExtTrigger(const StampBundle &stamp) = 0;

    virtual void visualizeStart() = 0;
    virtual void visualizeStop()  = 0;

    virtual void recordStart() {
        record_ = true;
    }

    virtual void recordStop() {
        record_ = false;
    }

    Type type() const {
        return type_;
    }

    std::string label() const {
        return label_;
    }

    bool requireThread() const {
        return require_thread_;
    }

    void setSynchronizer(const SynchronizerPtr &synchronizer) {
        synchronizer_ = synchronizer;
    }

protected:
    // 标志位
    std::atomic<bool> record_{false};

    // 属性
    Type type_{Type::INVALID};
    std::string label_;
    bool require_thread_{false};

    // 主系统指针
    SynchronizerPtr synchronizer_{nullptr};
};

} // namespace MSC