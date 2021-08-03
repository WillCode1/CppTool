#include "Singleton.h"

std::mutex Singleton::s_mutex;
Singleton* Singleton::s_pSingleton = nullptr;

Singleton* Singleton::getInstance()
{
	if (s_pSingleton == nullptr)
	{
		std::lock_guard<decltype(s_mutex)> lock(s_mutex);
		if (s_pSingleton == nullptr)
		{
			s_pSingleton = new Singleton();
		}
	}
	return s_pSingleton;
}

void Singleton::releaseInstance()
{
	if (s_pSingleton != nullptr)
	{
		std::lock_guard<decltype(s_mutex)> lock(s_mutex);
		if (s_pSingleton != nullptr)
		{
			delete s_pSingleton;
			s_pSingleton = nullptr;
		}
	}
}
