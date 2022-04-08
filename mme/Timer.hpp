//
// Created by will on 2022/2/17.
//

#pragma once

#include <chrono>
#include <iostream>

class Timer {
    using CalculatePrecision = std::chrono::microseconds;
    using ClockType = std::chrono::steady_clock;
public:
    void start() {
        t_last_ = t_start_ = std::chrono::steady_clock::now();
    }

    void reset() {
        start();
    }

    void elapsedByStart() {
        auto t_end = ClockType::now();
        auto start = std::chrono::time_point_cast<CalculatePrecision>(t_start_).time_since_epoch().count();
        auto end = std::chrono::time_point_cast<CalculatePrecision>(t_end).time_since_epoch().count();
        auto ms = 0.001 * (end - start);
        std::cout << "total by start:" << ms << "ms\n";
    }

    void elapsedByLast() {
        auto t_end = ClockType::now();
        auto last = std::chrono::time_point_cast<CalculatePrecision>(t_last_).time_since_epoch().count();
        auto end = std::chrono::time_point_cast<CalculatePrecision>(t_end).time_since_epoch().count();
        auto ms = 0.001 * (end - last);
        t_last_ = t_end;
        std::cout << "total by last:" << ms << "ms\n";
    }

private:
    std::chrono::time_point<ClockType> t_start_;
    std::chrono::time_point<ClockType> t_last_;
};
