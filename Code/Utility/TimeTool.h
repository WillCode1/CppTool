#pragma once
#ifndef _TIME_TOOL_H_
#define _TIME_TOOL_H_
#include <chrono>
#include <string>


/*
	default precision
	std::chrono::system_clock: microseconds
	std::chrono::steady_clock: nanoseconds
	std::chrono::high_resolution_clock: nanoseconds
 */
class TimeTool
{
public:
	using DefaultPrecision = std::chrono::milliseconds;

	template <typename Precision = DefaultPrecision>
	static long long getSystemTime(void)
	{
		return getCurTime<std::chrono::system_clock, Precision>();
	}
	template <typename Precision = DefaultPrecision>
	static long long getSteadyTime(void)
	{
		return getCurTime<std::chrono::steady_clock, Precision>();
	}
	template <typename Precision = std::chrono::nanoseconds>
	static long long getHighPrecisionTime(void)
	{
		return getCurTime<std::chrono::high_resolution_clock, Precision>();
	}

	// return duration time.
	template <typename Precision = DefaultPrecision>
	static long long durationTime(const long long& last)
	{
		return getCurTime<std::chrono::steady_clock, Precision>() - last;
	}

	static void threadDelayS(const long long& seconds);
	static void threadDelayMs(const long long& milliseconds);
	static void threadDelayUs(const long long& microseconds);
	static void threadDelayNs(const long long& nanoseconds);

	static char* getLocalTimeStamp();

private:
	template <typename ClockType>
	static std::chrono::time_point<ClockType> getTimePointNow(void)
	{
		return ClockType::now();
	}
	template <typename ClockType, typename Precision = DefaultPrecision>
	static long long durationCast(const std::chrono::time_point<ClockType>& time_point)
	{
		return std::chrono::duration_cast<Precision>(time_point.time_since_epoch()).count();
	}
	template <typename ClockType, typename Precision = DefaultPrecision>
	static long long getCurTime(void)
	{
		return durationCast<ClockType, Precision>(getTimePointNow<ClockType>());
	}
};

class Timer
{
public:
	enum TimerPrecision
	{
		Tp_Second,
		Tp_Millisecond,
		Tp_Microsecond,
		Tp_Nanosecond
	};

	Timer(bool print = false, TimerPrecision _timerPrecision = Tp_Millisecond);
	void StartTimer();
	double ElapsedByStart() const;
	double ElapsedByLast();

private:
	bool need_print;
	long long startTime;
	long long lastTime;
	TimerPrecision timerPrecision;
	std::string unit;
};

#endif
