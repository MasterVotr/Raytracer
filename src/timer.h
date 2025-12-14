#pragma once

#include <chrono>
#include <ostream>

class Timer {
    public:
        Timer();
        Timer(std::string name);
        inline double elapsed_s() const;
        inline double elapsed_ms() const;
        inline double elapsed_μs() const;
        inline uint64_t elapsed_ns() const;
        inline const std::string& name() const;
        inline void reset();
    private:
        std::chrono::high_resolution_clock::time_point start_;
        std::string name_;
};

Timer::Timer() : Timer("") {}

Timer::Timer(std::string name) : start_(std::chrono::high_resolution_clock::now()), name_(name) {}

double Timer::elapsed_s() const {
    return elapsed_ns() / 1000000000.0;
}

double Timer::elapsed_ms() const {
    return elapsed_ns() / 1000000.0;
}

double Timer::elapsed_μs() const {
    return elapsed_ns() / 1000.0;
}

uint64_t Timer::elapsed_ns() const {
    auto end = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(end - start_).count();
}

const std::string& Timer::name() const {
    return name_;
}

void Timer::reset() {
    start_ = std::chrono::high_resolution_clock::now();
}

