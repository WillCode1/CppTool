#include "TimeTool.h"
#include <chrono>
#include <ctime>
#include <thread>
#include <iostream>


void TimeTool::threadDelayS(const long long& seconds)
{
	std::this_thread::sleep_for(std::chrono::seconds(seconds));
}

void TimeTool::threadDelayMs(const long long& milliseconds)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

void TimeTool::threadDelayUs(const long long& microseconds)
{
	std::this_thread::sleep_for(std::chrono::microseconds(microseconds));
}

void TimeTool::threadDelayNs(const long long& nanoseconds)
{
	std::this_thread::sleep_for(std::chrono::nanoseconds(nanoseconds));
}


char* TimeTool::getLocalTimeStamp()
{
	static char buf[50];
	const static char *wday[] = { "Sun","Mon","Tue","Wed","Thu","Fri","Sat" };

	auto now = getTimePointNow<std::chrono::system_clock>();
	// 精确到毫秒
	int milliseconds = durationCast(now) % 1000;

	//获取系统日期和时间
	std::time_t now_st = std::chrono::system_clock::to_time_t(now);
	//ctime_s(buf, 100, &now);
	//std::cout << "now is: " << buf;

	//获取当地日期和时间
	struct tm local_t;
	localtime_s(&local_t, &now_st);

	snprintf(buf, 100, "[%04d-%02d-%02d %s %02d:%02d:%02d %03d] ",
		(1900 + local_t.tm_year), (1 + local_t.tm_mon), local_t.tm_mday, wday[local_t.tm_wday], 
		local_t.tm_hour, local_t.tm_min, local_t.tm_sec, milliseconds);
	return buf;
}


Timer::Timer(bool print, TimerPrecision _timerPrecision) : need_print(print), timerPrecision(_timerPrecision)
{
	switch (timerPrecision)
	{
	case Tp_Second:
		unit = "s";
		break;
	case Tp_Millisecond:
		unit = "ms";
		break;
	case Tp_Microsecond:
		unit = "us";
		break;
	case Tp_Nanosecond:
		unit = "ns";
		break;
	default:
		break;
	}
}

void Timer::StartTimer()
{
	switch (timerPrecision)
	{
	case Tp_Second:
		startTime = TimeTool::getSteadyTime<std::chrono::seconds>();
		lastTime = TimeTool::getSteadyTime<std::chrono::seconds>();
		break;
	case Tp_Millisecond:
		startTime = TimeTool::getSteadyTime();
		lastTime = TimeTool::getSteadyTime();
		break;
	case Tp_Microsecond:
		startTime = TimeTool::getSteadyTime<std::chrono::microseconds>();
		lastTime = TimeTool::getSteadyTime<std::chrono::microseconds>();
		break;
	case Tp_Nanosecond:
		startTime = TimeTool::getSteadyTime<std::chrono::nanoseconds>();
		lastTime = TimeTool::getSteadyTime<std::chrono::nanoseconds>();
		break;
	default:
		break;
	}
}

double Timer::ElapsedByStart() const
{
	long long total_time = 0;
	switch (timerPrecision)
	{
	case Tp_Second:
		total_time = TimeTool::durationTime<std::chrono::seconds>(lastTime);
		break;
	case Tp_Millisecond:
		total_time = TimeTool::durationTime(lastTime);
		break;
	case Tp_Microsecond:
		total_time = TimeTool::durationTime<std::chrono::microseconds>(lastTime);
		break;
	case Tp_Nanosecond:
		total_time = TimeTool::durationTime<std::chrono::nanoseconds>(lastTime);
		break;
	default:
		break;
	}
	if (need_print)
	{
		printf("Total took %lld%s.\n", total_time, unit.c_str());
	}
	return total_time;
}

double Timer::ElapsedByLast()
{
	long long total_time = 0;
	switch (timerPrecision)
	{
	case Tp_Second:
		total_time = TimeTool::durationTime<std::chrono::seconds>(lastTime);
		lastTime = TimeTool::getSteadyTime<std::chrono::seconds>();
		break;
	case Tp_Millisecond:
		total_time = TimeTool::durationTime(lastTime);
		lastTime = TimeTool::getSteadyTime();
		break;
	case Tp_Microsecond:
		total_time = TimeTool::durationTime<std::chrono::microseconds>(lastTime);
		lastTime = TimeTool::getSteadyTime<std::chrono::microseconds>();
		break;
	case Tp_Nanosecond:
		total_time = TimeTool::durationTime<std::chrono::nanoseconds>(lastTime);
		lastTime = TimeTool::getSteadyTime<std::chrono::nanoseconds>();
		break;
	default:
		break;
	}
	if (need_print)
	{
		printf("It took %lld%s.\n", total_time, unit.c_str());
	}
	return total_time;
}

