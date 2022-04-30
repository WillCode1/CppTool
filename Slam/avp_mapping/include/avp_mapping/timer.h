#pragma once

#include <chrono>
#include <iostream>
#include <string>

class Timer {
public:
  Timer(std::string timer_name);
  ~Timer();
  void Print();

  double GetTimeConsuming();

private:
  std::chrono::steady_clock::time_point start_timer_;
  std::string timer_name_;
  double ms_;
};
