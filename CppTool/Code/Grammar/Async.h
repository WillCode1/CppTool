#pragma once
#include <iostream> // std::cout, std::endl
#include <thread>   // std::thread
#include <string>   // std::string
#include <future>   // std::promise, std::future
#include <chrono>   // seconds

namespace cpp11 {
	using namespace std;
	using namespace std::chrono;

	// https://www.jianshu.com/p/7945428c220e
	void read(std::future<std::string> *future) {
		// future会一直阻塞，直到有值到来
		std::cout << future->get() << std::endl;
	}

	void test_promise() {
		// promise 相当于生产者
		std::promise<std::string> promise;
		// future 相当于消费者, 右值构造
		std::future<std::string> future = promise.get_future();
		// 另一线程中通过future来读取promise的值
		std::thread thread(read, &future);
		// 让read等一会儿:)
		std::this_thread::sleep_for(seconds(1));
		
		promise.set_value("hello future");
		// 等待线程执行完成
		thread.join();
	}

	/****************************************/
	// https://www.jianshu.com/p/72601d82f3df
	int sum(int a, int b) {
		return a + b;
	}

	void test_packaged_task() {
		std::packaged_task<int(int, int)> task(sum);
		std::future<int> future = task.get_future();

		// std::promise一样，std::packaged_task支持move，但不支持拷贝
		// std::thread的第一个参数不止是函数，还可以是一个可调用对象，即支持operator()(Args...)操作
		std::thread t(std::move(task), 1, 2);
		// 等待异步计算结果
		std::cout << "1 + 2 => " << future.get() << std::endl;

		t.join();
	}

	/****************************************/
	// https://www.jianshu.com/p/58dea28d1a95
	void test_async()
	{
		auto print = [](char c) {
			for (int i = 0; i < 10; i++) {
				std::cout << c;
				std::cout.flush();
				std::this_thread::sleep_for(milliseconds(1));
			}
		};
		// 不同launch策略的效果
		std::launch policies[] = { std::launch::async, std::launch::deferred };
		const char *names[] = { "async   ", "deferred" };
		for (int i = 0; i < sizeof(policies) / sizeof(policies[0]); i++) {
			std::cout << names[i] << ": ";
			std::cout.flush();
			auto f1 = std::async(policies[i], print, '+');
			auto f2 = std::async(policies[i], print, '-');
			f1.get();
			f2.get();
			std::cout << std::endl;
		}
	}
};
