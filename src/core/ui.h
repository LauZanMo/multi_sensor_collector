#pragma once

#include "absl/strings/str_cat.h"
#include <absl/strings/str_format.h>
#include <memory>
#include <tbb/concurrent_queue.h>
#include <yaml-cpp/yaml.h>

struct _win_st;

namespace MSC {

class Synchronizer;
typedef std::shared_ptr<Synchronizer> SynchronizerPtr;

class UI {
public:
    typedef std::shared_ptr<UI> Ptr;

    explicit UI(const YAML::Node &config);
    ~UI();

    void setSynchroinzer(const SynchronizerPtr &synchronizer) {
        synchronizer_ = synchronizer;
    }

    void init();
    void processKeyInput();
    void processLog();
    void processViz();

    template <typename T, typename... Ts>
    void addLog(const T &first, const Ts &...rest) {
        level_queue_.push(first);
        std::string log = absl::StrCat(rest...);
        log_queue_.emplace(std::move(log));
    }

    template <typename T, typename... Ts>
    void addViz(const T &first, const Ts &...rest) {
        std::string viz = absl::StrCat(first, rest...);
        viz_queue_.emplace(std::move(viz));
    }

private:
    _win_st *createWindow(const std::string &name, double start_x, double start_y, double width, double height);
    void parse(const std::string &cmd);

    SynchronizerPtr synchronizer_;
    _win_st *log_win_, *input_win_, *viz_win_;

    // 容器
    tbb::concurrent_queue<std::string> log_queue_, viz_queue_, level_queue_;

    // 属性
    int width_, height_;
};

} // namespace MSC