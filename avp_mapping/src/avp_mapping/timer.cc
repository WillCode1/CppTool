#include "timer.h"

Timer::Timer(std::string timer_name) : timer_name_(timer_name), ms_(0) {
  start_timer_ = std::chrono::steady_clock::now();
}

void Timer::Print() {
  std::chrono::steady_clock::time_point end_time =
      std::chrono::steady_clock::now();

  double time_duration =
      std::chrono::duration_cast<std::chrono::duration<double>>(end_time -
                                                                start_timer_)
          .count();

  ms_ = time_duration * 1000.0f;
  std::cout << " Timer(" << timer_name_ << ") : " << time_duration * 1000.0f
            << " ms " << std::endl;
}

double Timer::GetTimeConsuming() { return ms_; }

Timer::~Timer() {}
