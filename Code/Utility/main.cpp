// Utility.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include <direct.h>
#include "TestTSingleton.h"
#include "FileTool.h"
#include "StringTool.h"
#include "RunTimeTool.h"
#include "LogTool.h"
#include "TimeTool.h"
#include "Random.hpp"
#include <assert.h>

using namespace std;

#define NDEBUG

int main()
{
	RunTimeTool test;
	test.StartTimer();

	//TestJson::JsonBase();

	auto& people = SinglePeople::GetInstance();
	people.SetAge(18);
	auto age = people.SayAge();
	SinglePeople::ReleaseInstance();

#if 0
	for (int i = 0; i < 20; ++i)
	{
		cout << Random::RandInt(2, 5) << endl;
		cout << Random::RandDouble(-2.5, 3.5) << endl;
	}
#endif

	test.ElapsedTime();

	std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() << std::endl;
	std::cout << std::chrono::steady_clock::now().time_since_epoch().count() << std::endl;

	std::cout << getUseconds() << std::endl;
	std::cout << std::chrono::system_clock::now().time_since_epoch().count() << std::endl;

	log_print(error, "asd %d", 1);
	
	//assert(false);

	return 0;
}

