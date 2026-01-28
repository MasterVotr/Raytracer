#pragma once

#include <chrono>
#include <ostream>

class Timer {
   public:
    Timer() : Timer("") {}
    Timer(std::string name) : start_(std::chrono::high_resolution_clock::now()), name_(name) {}

    inline double elapsed_s() const { return elapsed_ns() / 1'000'000'000.0; }

    inline double elapsed_ms() const { return elapsed_ns() / 1'000'000.0; }

    inline double elapsed_μs() const { return elapsed_ns() / 1'000.0; }

    inline uint64_t elapsed_ns() const {
        auto end = std::chrono::high_resolution_clock::now();
        return std::chrono::duration_cast<std::chrono::nanoseconds>(end - start_).count();
    }

    inline const std::string& name() const { return name_; }

    inline void reset() { start_ = std::chrono::high_resolution_clock::now(); }

   private:
    std::chrono::high_resolution_clock::time_point start_;
    std::string name_;
};
