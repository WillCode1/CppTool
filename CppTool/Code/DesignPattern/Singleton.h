#pragma once
#include <mutex>

/// @brief 线程安全的懒汉模式
class Singleton
{
public:
	static Singleton* getInstance();
	static void releaseInstance();

private:
	Singleton() = default;
	Singleton(const Singleton&) = delete;
	Singleton(const Singleton&&) = delete;
	Singleton& operator=(const Singleton&) = delete;
	Singleton& operator=(const Singleton&&) = delete;

	static Singleton* s_pSingleton;
	static std::mutex s_mutex;
};
