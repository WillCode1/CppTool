#include "SystemApi.h"
#include <thread>
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif // _WIN32


void sleep(long milliseconds)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));

#ifdef _WIN32
	Sleep(milliseconds);
#else
	usleep(milliseconds * 1000);
#endif // _WIN32
}

