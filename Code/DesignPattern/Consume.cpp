#include "Consume.h"
#include <iostream>
#include <thread>
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif // _WIN32

BlockingQueue<int> g_queue;


// 生产者线程
void Produce()
{
	for (int i = 0; i < 10; ++i)
	{
		g_queue.push(i);
		std::cout << "Produce：" << i << std::endl;
	}
}

// 消费者线程
void Consume()
{
	int data;
	while (true)
	{
		PopResult res = g_queue.pop(data);
		if (res == POP_STOP) // 线程应该停止
			break;
		if (res == POP_UNEXPECTED) // 意外唤醒
			continue;

		// 处理数据
		std::cout << "Consume：" << data << std::endl;
	}
}

int testConsume()
{
	// 启动生产者线程和消费者线程（也可以启动多个线程）
	std::thread produceThread(Produce);
	std::thread consumerThread(Consume);

	// 等待数据处理完
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
#ifdef _WIN32
	Sleep(1000);
#else
	usleep(1000 * 1000);
#endif // _WIN32

	produceThread.join();
	// 停止线程
	g_queue.Stop();
	consumerThread.join();

	return 0;
}
