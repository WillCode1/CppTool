// Utility.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include <cassert>
#include "TestTSingleton.h"
#include "FileTool.h"
#include "StringTool.h"
#include "LogTool.h"
#include "TimeTool.hpp"
#include "Timer.hpp"
#include "Random.hpp"

using namespace std;

#define NDEBUG


int main()
{
	Timer test;
	test.start();

	TimeTool::ThreadDelayMs(1000);
	//TestJson::JsonBase();

	auto& people = SinglePeople::GetInstance(0, 24, "Will");
	auto age = people.SayAge();
	SinglePeople::ReleaseInstance();

#if 1
	for (int i = 0; i < 20; ++i)
	{
		cout << Random::RandInt(2, 5) << endl;
		cout << Random::RandDouble(-2.5, 3.5) << endl;
	}
#endif

	test.elapsedByStart();
	test.elapsedByLast();

	log_print(error, "asd %d", 1);
	
	//assert(false);

	return 0;
}

