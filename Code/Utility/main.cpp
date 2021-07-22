// Utility.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include <direct.h>
#include "TestTSingleton.h"
#include "FileTool.h"
#include "StringTool.h"
#include "DebugTool.hpp"
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

	string a = "asd";
	wstring b = L"ADAS";
	DEBUG_LOG(11);
	DEBUG_LOG(1.12);
	DEBUG_LOG('a');
	DEBUG_LOG("asdsad");
	DEBUG_LOG(L"asdsad");
	DEBUG_LOG(a);
	DEBUG_LOG(b);

#if 0
	for (int i = 0; i < 20; ++i)
	{
		cout << Random::RandInt(2, 5) << endl;
		cout << Random::RandDouble(-2.5, 3.5) << endl;
	}
#endif

	test.ElapsedTime();

	log_print(error, "asd %d", 1);
	
	assert(false);

	return 0;
}

