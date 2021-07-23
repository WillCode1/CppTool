#pragma once
#include <chrono>
#include <string>
#include <ctime>
#include <thread>
#include <iostream>

/*
	default precision
	std::chrono::system_clock: microseconds
	std::chrono::steady_clock: nanoseconds
	std::chrono::high_resolution_clock: nanoseconds
 */
class TimeTool
{
	using DefaultPrecision = std::chrono::milliseconds;

public:
	template <typename Precision = DefaultPrecision>
	static long long GetSystemTime(void)
	{
		return GetCurTime<std::chrono::system_clock, Precision>();
	}
	template <typename Precision = DefaultPrecision>
	static long long GetSteadyTime(void)
	{
		return GetCurTime<std::chrono::steady_clock, Precision>();
	}
	template <typename Precision = std::chrono::nanoseconds>
	static long long GetHighPrecisionTime(void)
	{
		return GetCurTime<std::chrono::high_resolution_clock, Precision>();
	}

	// return duration time.
	template <typename Precision = DefaultPrecision>
	static long long DurationTime(const long long& last)
	{
		return GetSteadyTime<Precision>() - last;
	}

	static void ThreadDelayS(const long long& seconds)
	{
		std::this_thread::sleep_for(std::chrono::seconds(seconds));
	}
	static void ThreadDelayMs(const long long& milliseconds)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
	}
	static void ThreadDelayUs(const long long& microseconds)
	{
		std::this_thread::sleep_for(std::chrono::microseconds(microseconds));
	}
	static void ThreadDelayNs(const long long& nanoseconds)
	{
		std::this_thread::sleep_for(std::chrono::nanoseconds(nanoseconds));
	}

	static char* GetLocalTimeStamp()
	{
		static char buf[50];
		const static char *wday[] = { "Sun","Mon","Tue","Wed","Thu","Fri","Sat" };

		auto now = GetCurTimePoint<std::chrono::system_clock>();

		//获取系统日期和时间
		std::time_t now_st = std::chrono::system_clock::to_time_t(now);
		//ctime_s(buf, 100, &now);
		//std::cout << "now is: " << buf;

		//获取当地日期和时间
		struct tm local_t;
		localtime_s(&local_t, &now_st);

		// 精确到毫秒
		int milliseconds = PrecisionCast(now) % 1000;

		snprintf(buf, 100, "[%04d-%02d-%02d %s %02d:%02d:%02d %03d] ",
			(1900 + local_t.tm_year), (1 + local_t.tm_mon), local_t.tm_mday, wday[local_t.tm_wday],
			local_t.tm_hour, local_t.tm_min, local_t.tm_sec, milliseconds);
		return buf;
	}

private:
	template <typename ClockType>
	static std::chrono::time_point<ClockType> GetCurTimePoint(void)
	{
		return ClockType::now();
	}
	template <typename ClockType, typename Precision = DefaultPrecision>
	static long long PrecisionCast(const std::chrono::time_point<ClockType>& time_point)
	{
		return std::chrono::duration_cast<Precision>(time_point.time_since_epoch()).count();
	}
	template <typename ClockType, typename Precision = DefaultPrecision>
	static long long GetCurTime(void)
	{
		return PrecisionCast<ClockType, Precision>(GetCurTimePoint<ClockType>());
	}
};

