#include "core/serial_helper.h"
#include "core/logger.h"

#include <filesystem>
#include <thread>

namespace MSC {

void openSerial(serial::Serial &serial, const SerialParam &param, const std::string &prefix) {
    while (!std::filesystem::exists(param.port)) {
        LOGI << prefix << ": Waiting for serial port " << param.port << " to be available...";
        std::this_thread::sleep_for(std::chrono::seconds(1));
        // TODO: 判断线程结束跳出循环
    }

    try {
        serial.setPort(param.port);
        serial.setBaudrate(param.baudrate);
        serial.setTimeout(param.timeout);
        serial.open();
    } catch (serial::IOException &e) {
        LOGE << prefix << ": Failed to open serial port " << param.port << ": " << e.what();
        return;
    }

    if (serial.isOpen()) {
        LOGI << prefix << ": Serial port " << param.port << " initialized.";
    }
}

} // namespace MSC