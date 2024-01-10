#include "absl/strings/str_cat.h"
#include "core/logger.h"
#include "core/synchronizer.h"

#include <absl/strings/str_format.h>
#include <absl/time/clock.h>
#include <filesystem>
#include <iostream>
#include <yaml-cpp/yaml.h>

std::string msc_config_file = "../config/suite.yaml";

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

    auto synchronizer = std::make_shared<MSC::Synchronizer>(msc_config);
    auto data_writer  = std::make_shared<MSC::DataWriter>(msc_config);

    synchronizer->setDataWriter(data_writer);
    data_writer->setSynchronizer(synchronizer);

    synchronizer->init();
    synchronizer->process();

    // MSC::UI ui(msc_config);
    // ui.init();
    // std::thread ui_thread1(&MSC::UI::processLog, &ui);
    // std::thread ui_thread2(&MSC::UI::processViz, &ui);
    // ui.processKeyInput();
    //
    // if (ui_thread1.joinable())
    //     ui_thread1.join();
    //
    // if (ui_thread2.joinable())
    //     ui_thread2.join();

    MSC::Logger::shutdown();
    return 0;
}
