#include <chrono>
#include <time.h>

#ifdef _WIN32
#include <windows.h>
#include <winsock.h>
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
#else
#include <sys/time.h>
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

/*delay usec: the input usec is 0<= usec <= 1000*1000*/
bool threadDelayUs(const long usec)
{
	struct timeval delay;

	if (usec < 0 || usec>1000 * 1000) {
		printf("Usec value invalid: %ld\n", usec);
		return false;
	}

	delay.tv_sec = 0;
	delay.tv_usec = usec;
	if (0 != select(0, NULL, NULL, NULL, &delay)) {
		return false;
	}
	else {
		return true;
	}
}

/*delay msec: the input msec is 0<= msec <= 1000*/
bool threadDelayMs(const long msec)
{
	struct timeval delay;

	if (msec < 0 || msec>10000) {
		printf("Msec value invalid: %ld\n", msec);
		return false;
	}

	delay.tv_sec = 0;
	delay.tv_usec = 1000 * msec;
	if (0 != select(0, NULL, NULL, NULL, &delay)) {
		return false;
	}
	else {
		return true;
	}
}

/*delay sec: the input sec is 0<= sec <= 100*/
bool threadDelayS(const long sec)
{
	struct timeval delay;

	if (sec < 0 || sec>100) {
		printf("Delay sec value invalid: %ld\n", sec);
		return false;
	}

	delay.tv_sec = sec;
	delay.tv_usec = 0;
	if (0 != select(0, NULL, NULL, NULL, &delay)) {
		return false;
	}
	else {
		return true;
	}
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

