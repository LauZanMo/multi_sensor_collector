#pragma once

#include <yaml-cpp/yaml.h>

#include "core/sensor_base.h"
#include "core/serial_helper.h"

namespace MSC {

#pragma pack(push, 1)
struct InsProbeIMU {
    uint16_t week;      ///< GPS周
    double week_second; ///< GPS周内秒
    float gyro_x;       ///< 陀螺x
    float gyro_y;       ///< 陀螺y
    float gyro_z;       ///< 陀螺z
    float acc_x;        ///< 加表x
    float acc_y;        ///< 加表y
    float acc_z;        ///< 加表z
    float reserved1;    ///< 预留1
    int16_t odom;       ///< 里程计
    uint8_t reserved2;  ///< 预留2
    uint8_t reserved3;  ///< 预留3
};
#pragma pack(pop)
static const uint8_t packet_ID[2] = {0x55, 0xAA};

class InsProbe : public SensorBase {
public:
    InsProbe(const YAML::Node &config, const YAML::Node &sensor_config);

    void init() override;
    void process() override;

    void insertExtTrigger(const StampBundle &stamp) override;

    void visualizeStart() override;
    void visualizeStop() override;

private:
    bool parse(const uint8_t &data);

    SerialParam serial_param_;
    serial::Serial serial_;

    uint8_t counter_{0}, checksum_{0};
    InsProbeIMU insprobe_imu_;

    std::atomic<bool> visualize_{false};

    static constexpr double DEG2RAD = M_PI / 180.0;
};

} // namespace MSC