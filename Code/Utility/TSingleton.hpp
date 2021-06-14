// µ¥ÀýÄ£°åÀà
#pragma once
#include <mutex>

template <class T>
class TSingleton
{
public:
	virtual ~TSingleton() {}

	static T& GetInstance()
	{
		if (s_pInsatcne == nullptr)
		{
			std::lock_guard<decltype(s_mutex)> lock(s_mutex);
			if (s_pInsatcne == nullptr)
			{
				s_pInsatcne = new T();
			}
		}

		return *s_pInsatcne;
	}

	static void ReleaseInstance()
	{
		if (nullptr != s_pInsatcne)
		{
			std::lock_guard<decltype(s_mutex)> lock(s_mutex);
			if (nullptr != s_pInsatcne)
			{
				delete s_pInsatcne;
				s_pInsatcne = nullptr;
			}
		}
	}
protected:
	TSingleton() = default;
private:
	TSingleton(const TSingleton&) = delete;
	TSingleton(const TSingleton&&) = delete;
	TSingleton& operator=(const TSingleton&) = delete;
	TSingleton& operator=(const TSingleton&&) = delete;
private:
	static T*			s_pInsatcne;
	static std::mutex	s_mutex;
};

template <class T>
T* TSingleton<T>::s_pInsatcne = nullptr;
template <class T>
std::mutex TSingleton<T>::s_mutex;
