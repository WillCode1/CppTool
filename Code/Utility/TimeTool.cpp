#include <chrono>
#include <time.h>
#include <thread>
#ifndef _WIN32
#include <sys/time.h>
#else
#include <windows.h>
#include <winsock.h>
#include "TimeTool.h"
#pragma comment(lib, "Ws2_32.lib")
#pragma comment(lib, "Wldap32.lib")


int gettimeofday(struct timeval *tp, void *tzp)
{
	time_t clock;
	struct tm tm;
	SYSTEMTIME wtm;

	GetLocalTime(&wtm);
	tm.tm_year = wtm.wYear - 1900;
	tm.tm_mon = wtm.wMonth - 1;
	tm.tm_mday = wtm.wDay;
	tm.tm_hour = wtm.wHour;
	tm.tm_min = wtm.wMinute;
	tm.tm_sec = wtm.wSecond;
	tm.tm_isdst = -1;
	clock = mktime(&tm);
	tp->tv_sec = clock;
	tp->tv_usec = wtm.wMilliseconds * 1000;

	return 0;
}
#endif // _WIN32



long long getMseconds(void)
{
	auto start = std::chrono::steady_clock::now();
	long long end = std::chrono::duration_cast<std::chrono::milliseconds>(
		start.time_since_epoch()).count();
	return end;
}

long long getUseconds(void)
{
	struct timeval t;
	gettimeofday(&t, NULL);
	return ((long long)t.tv_sec) * 1000000 + (long long)t.tv_usec;
}

void threadDelayS(const long long& seconds)
{
	std::this_thread::sleep_for(std::chrono::seconds(seconds));
}

void threadDelayMs(const long long& milliseconds)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

void threadDelayUs(const long long& microseconds)
{
	std::this_thread::sleep_for(std::chrono::microseconds(microseconds));
}

void threadDelayNs(const long long& nanoseconds)
{
	std::this_thread::sleep_for(std::chrono::nanoseconds(nanoseconds));
}

char *getTime()
{
	static char buf[100];
	const char *wday[] = { "Sun","Mon","Tue","Wed","Thu","Fri","Sat" };

	struct tm t;
	time_t now;
	time(&now);					//获取系统日期和时间
	localtime_s(&t, &now);		//获取当地日期和时间
	struct timeval nowtimeval;
	gettimeofday(&nowtimeval, 0);

	snprintf(buf, 100, "[%04d-%02d-%02d %s %02d:%02d:%02d %03d] ",
		(1900 + t.tm_year), (1 + t.tm_mon), t.tm_mday, wday[t.tm_wday], t.tm_hour, t.tm_min, t.tm_sec, nowtimeval.tv_usec / 1000);
	return buf;
}

bool getLocalTime(int* h, int* m)
{
	struct tm t;
	time_t now;
	time(&now);					//获取系统日期和时间
	localtime_s(&t, &now);		//获取当地日期和时间
	*h = t.tm_hour;
	*m = t.tm_min;
	//printf("get localtime: %d-%d-%d\n",1900+t.tm_year,t.tm_mon,t.tm_mday);//
	//printf("week %d time %d:%d:%d\n",t.tm_wday,t.tm_hour,t.tm_min,t.tm_sec);//	
	int thisyear = t.tm_year + 1900;
	if (thisyear < 2017 || thisyear>2025)
	{
		return false;
	}
	return true;
}

