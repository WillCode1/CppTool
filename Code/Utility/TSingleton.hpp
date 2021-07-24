// µ¥ÀýÄ£°åÀà
#pragma once
#include <mutex>

template <class T> class TSingleton
{
public:
	virtual ~TSingleton() = default;

	template<typename ...Args>
	static T& GetInstance(Args&&... args)
	{
		if (nullptr == s_pInsatcne)
		{
			std::lock_guard<decltype(s_mutex)> lock(s_mutex);
			if (nullptr == s_pInsatcne)
			{
				s_pInsatcne = std::make_shared<T>(std::forward<Args>(args)...);
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
				s_pInsatcne.reset();
			}
		}
	}

protected:
	explicit TSingleton() = default;

private:
	TSingleton(const TSingleton&) = delete;
	TSingleton(const TSingleton&&) = delete;
	TSingleton& operator=(const TSingleton&) = delete;
	TSingleton& operator=(const TSingleton&&) = delete;

private:
	static std::shared_ptr<T>	s_pInsatcne;
	static std::mutex			s_mutex;
};

template <class T> std::shared_ptr<T> TSingleton<T>::s_pInsatcne = nullptr;
template <class T> std::mutex TSingleton<T>::s_mutex;
