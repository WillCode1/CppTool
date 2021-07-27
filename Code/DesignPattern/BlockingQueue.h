// https://blog.csdn.net/xfgryujk/article/details/70970226

#pragma once
#include <queue>
#include <mutex>
#include <condition_variable>


enum PopResult { POP_OK, POP_STOP, POP_EMPTY };

template<class T>
class BlockingQueue
{
public:
	bool m_stopFlag = false;
	std::mutex m_lock;
	std::condition_variable m_cond;
	std::queue<T> m_queue;

	virtual ~BlockingQueue() = default;

	void push(const T& value)
	{
		std::lock_guard<decltype(m_lock)> lock(m_lock);
		m_queue.push(value);
		m_cond.notify_one();
	}

	void push(T&& value)
	{
		std::lock_guard<decltype(m_lock)> lock(m_lock);
		m_queue.push(std::move(value));
		m_cond.notify_one();
	}

	PopResult pop(T& out)
	{
		std::unique_lock<decltype(m_lock)> lock(m_lock);
		if (m_stopFlag)
			return POP_STOP;
		if (m_queue.empty())
			m_cond.wait(lock);

		if (m_stopFlag)
			return POP_STOP;

		out = std::move(m_queue.front());
		m_queue.pop();
		return POP_OK;
	}

	PopResult try_pop(T& out)
	{
		std::unique_lock<decltype(m_lock)> lock(m_lock);
		if (m_stopFlag)
			return POP_STOP;
		if (m_queue.empty())
			return POP_EMPTY;

		out = std::move(m_queue.front());
		m_queue.pop();
		return POP_OK;
	}

	bool empty()
	{
		std::unique_lock<decltype(m_lock)> lock(m_lock);
		return m_queue.empty();
	}

	void stop()
	{
		std::lock_guard<decltype(m_lock)> lock(m_lock);
		m_stopFlag = true;
		m_cond.notify_all();
	}
};

int testConsume();
