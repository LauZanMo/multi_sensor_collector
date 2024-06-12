#include "core/ui.h"
#include "core/logger.h"
#include "core/synchronizer.h"

namespace MSC {

UI::UI(const YAML::Node &config) {
}

UI::~UI() {
}

void UI::init() {
}

void UI::processKeyInput() {
}

void UI::processLog() {
}

void UI::processViz() {
    std::string viz;
    while (synchronizer_->isRunning()) {
        if (viz_queue_.try_pop(viz)) {
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
}

void UI::parse(const std::string &cmd) {
    if (cmd == "exit") {
        synchronizer_->shutdown();
    } else if (cmd == "help") {
        addLog("info", "Available commands:\n");
        addLog("info", "  exit: exit the program\n");
        addLog("info", "  help: print this message\n");
    } else {
        addLog("info", "Unknown command: ", cmd, "\n");
    }
}

} // namespace MSC