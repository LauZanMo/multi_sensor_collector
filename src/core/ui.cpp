#include "core/ui.h"
#include "core/logger.h"
#include "core/synchronizer.h"

#include <ncursesw/ncurses.h>

// sudo add-apt-repository ppa:dankamongmen/notcurses
// sudo apt-get update
// sudo apt install libnotcurses-dev

namespace MSC {

UI::UI(const YAML::Node &config) {
}

UI::~UI() {
    delwin(log_win_);
    delwin(input_win_);
    delwin(viz_win_);
    endwin();
}

void UI::init() {
    // 初始化TUI
    setlocale(LC_ALL, ""); // 设置locale
    initscr();

    start_color();
    init_pair(1, COLOR_GREEN, COLOR_BLACK);
    init_pair(2, COLOR_YELLOW, COLOR_BLACK);
    init_pair(3, COLOR_RED, COLOR_BLACK);
    init_pair(4, COLOR_MAGENTA, COLOR_BLACK);

    getmaxyx(stdscr, height_, width_);
    log_win_   = createWindow("Log", 0.0, 0.0, 1.0, 0.5);
    input_win_ = createWindow("Input", 0.0, 0.5, 0.5, 0.5);
    viz_win_   = createWindow("Viz", 0.5, 0.5, 0.5, 0.5);
}

void UI::processKeyInput() {
    char cmd[1024];
    while (1) {
        // while (synchronizer_->isRunning()) {
        wprintw(input_win_, ">");
        if (wgetstr(input_win_, cmd) != ERR) {
            parse(cmd);
        }
    }
}

void UI::processLog() {
    std::string log, level;
    while (1) {
        // while (synchronizer_->isRunning()) {
        if (log_queue_.try_pop(log)) {
            level_queue_.try_pop(level);

            if (level == "info") {
                wattron(log_win_, COLOR_PAIR(1));
                wprintw(log_win_, log.data());
                wrefresh(log_win_);
                // LOGI << log;
            } else if (level == "warning") {
                wattron(log_win_, COLOR_PAIR(2));
                wprintw(log_win_, log.data());
                wrefresh(log_win_);
                // LOGW << log;
            } else if (level == "error") {
                wattron(log_win_, COLOR_PAIR(3));
                wprintw(log_win_, log.data());
                wrefresh(log_win_);
                // LOGE << log;
            } else if (level == "fatal") {
                wattron(log_win_, COLOR_PAIR(4));
                wprintw(log_win_, log.data());
                wrefresh(log_win_);
                // LOGF << log;
            }

        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
}

void UI::processViz() {
    std::string viz;
    while (1) {
        // while (synchronizer_->isRunning()) {
        if (viz_queue_.try_pop(viz)) {
            wprintw(viz_win_, viz.data());
            wrefresh(viz_win_);
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
}

_win_st *UI::createWindow(const std::string &name, double start_x, double start_y, double width, double height) {
    int x = static_cast<int>(start_x * width_);
    int y = static_cast<int>(start_y * height_);
    int w = static_cast<int>(width * width_);
    int h = static_cast<int>(height * height_);

    auto win = subwin(stdscr, h, w, y, x);
    scrollok(win, true);
    wsetscrreg(win, 0, h);
    wrefresh(win);

    return win;
}

void UI::parse(const std::string &cmd) {
    if (cmd == "exit") {
        synchronizer_->shutdown();
    } else if (cmd == "help") {
        addLog("info", "Available commands:\n");
        addLog("info", "  exit: exit the program\n");
        addLog("info", "  help: print this message\n");
        // addViz("Hello world!\n");
    } else {
        addLog("info", "Unknown command: ", cmd, "\n");
        // addViz("NO world!\n");
    }
}

} // namespace MSC