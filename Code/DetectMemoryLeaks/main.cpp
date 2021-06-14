// DetectMemoryLeaks.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
//#include <stdlib.h>

// 内存泄漏检测
#define _CRTDBG_MAP_ALLOC
#include <crtdbg.h>


void GetMemory(char *p, int num)
{
	p = static_cast<char*>(malloc(sizeof(char) * num));

	//使用new也能够检测出来
	auto c = new int(100);
}

int main()
{
	// 前置声明检测
	//_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
	
	// 断点
	_CrtSetBreakAlloc(158);
	_CrtSetBreakAlloc(159);

	char *str = nullptr;
	GetMemory(str, 100);
	std::cout << "Memory leak test!" << std::endl;

	_CrtDumpMemoryLeaks();
	return 0;
}
