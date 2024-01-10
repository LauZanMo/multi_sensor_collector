#pragma once

#include <serial/serial.h>

namespace MSC {

struct SerialParam {
    std::string port;
    uint32_t baudrate;
    serial::Timeout timeout;
    serial::bytesize_t bytesize;
    serial::parity_t parity;
    serial::stopbits_t stopbits;
    serial::flowcontrol_t flowcontrol;
};

void openSerial(serial::Serial &serial, const SerialParam &param, const std::string &prefix);

} // namespace MSC