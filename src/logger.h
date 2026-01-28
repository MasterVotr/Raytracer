#pragma once

#include <chrono>
#include <format>
#include <iostream>
#include <mutex>
#include <string_view>

class Logger {
   public:
    enum class LogLevel { Error = 2, Info = 1, Debug = 0 };
    static void init(LogLevel log_level = LogLevel::Debug) { log_level_ = log_level; }

    template <typename... Args>
    static void info(std::format_string<Args...> fmt, Args&&... args) {
        if (!should_log(LogLevel::Info)) return;
        log("INFO", fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    static void debug(std::format_string<Args...> fmt, Args&&... args) {
        if (!should_log(LogLevel::Debug)) return;
        log("DEBUG", fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    static void error(std::format_string<Args...> fmt, Args&&... args) {
        if (!should_log(LogLevel::Error)) return;
        log("ERROR", fmt, std::forward<Args>(args)...);
    }

   private:
    inline static std::mutex mtx_;
    inline static LogLevel log_level_;

    static bool should_log(LogLevel level) { return static_cast<int>(level) >= static_cast<int>(log_level_); }

    template <typename... Args>
    static void log(std::string_view level, std::format_string<Args...> fmt, Args&&... args) {
        auto now = std::chrono::system_clock::now();
        std::string msg = std::format(fmt, std::forward<Args>(args)...);
        std::lock_guard<std::mutex> lock(mtx_);
        std::cout << std::format("{:%F %T} [{}] {}\n", now, level, msg);
    }
};