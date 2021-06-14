#pragma once
#ifndef THREAD_POOL_H
#define THREAD_POOL_H

#include <vector>
#include <queue>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <future>
#include <functional>
#include <stdexcept>

namespace std
{
#define  MIN_THREAD_NUM 1
#define  MAX_THREAD_NUM 16

	//线程池,可以提交变参函数或lambda表达式的匿名函数执行,可以获取执行返回值
	//不支持类成员函数, 支持类静态成员函数或全局函数,Opteron()函数等
	class ThreadPool
	{
		using Task = std::function<void()>;
		// 线程池
		std::vector<std::thread> _pool;
		// 任务队列
		std::queue<Task> _tasks;
		// 同步
		std::mutex _lock;
		// 条件阻塞
		std::condition_variable _cv;
		// 是否关闭提交
		std::atomic<bool> _run{ true };
		//空闲线程数量
		std::atomic<int> _idlThrNum{ 0 };

	public:
		ThreadPool(unsigned char size = 4)
		{
			_idlThrNum = size < MIN_THREAD_NUM ? MIN_THREAD_NUM : size;
			_idlThrNum = size > MAX_THREAD_NUM ? MAX_THREAD_NUM : size;

			for (size = 0; size < _idlThrNum; ++size)
			{
				_pool.emplace_back(
					[this]
				{ // 工作线程函数
					while (_run)
					{
						Task task;
						{   // 获取一个待执行的 task
							std::unique_lock<std::mutex> lock{ _lock };// unique_lock 相比 lock_guard 的好处是：可以随时 unlock() 和 lock()
							_cv.wait(lock, [this] { return !_run.load() || !_tasks.empty(); }); // wait 直到有 task
							if (!_run && _tasks.empty())
								return;
							task = std::move(_tasks.front()); // 取一个 task
							_tasks.pop();
						}
						_idlThrNum--;
						task();
						_idlThrNum++;
					}
				}
				);
			}
		}
		~ThreadPool()
		{
			_run.store(false);
			_cv.notify_all(); // 唤醒所有线程执行
			for (auto& thread : _pool) {
				//thread.detach(); // 让线程“自生自灭”
				if (thread.joinable())
					thread.join(); // 等待任务结束， 前提：线程一定会执行完
			}
		}

	public:
		// 提交一个任务
		// 调用.get()获取返回值会等待任务执行完,获取返回值
		// 有两种方法可以实现调用类成员函数：
		// 一种是使用 bind： .commit(std::bind(&Dog::sayHello, &dog));
		// 一种是使用 mem_fn： .commit(std::mem_fn(&Dog::sayHello), &dog)
		template<class F, class... Args>
		auto commit(F&& f, Args&&... args) ->std::future<decltype(f(args...))>
		{
			if (!_run.load())
				throw std::runtime_error("commit on ThreadPool is stopped.");

			using FunctionType = decltype(f(args...)); // typename std::result_of<F(Args...)>::type, 函数 f 的返回值类型
			auto task = std::make_shared<std::packaged_task<FunctionType()>>(std::bind(std::forward<F>(f), std::forward<Args>(args)...));    // wtf !
			std::future<FunctionType> retType = task->get_future();
			{
				std::lock_guard<std::mutex> lock{ _lock };//对当前块的语句加锁  lock_guard 是 mutex 的 stack 封装类，构造的时候 lock()，析构的时候 unlock()
				_tasks.emplace([task]() {(*task)(); });
			}
			_cv.notify_one(); // 唤醒一个线程执行
			return retType;
		}

		//空闲线程数量
		int idlCount() { return _idlThrNum.load(); }
	};
}

#endif