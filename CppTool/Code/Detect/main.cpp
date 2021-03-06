// main.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include "DetectMemoryLeaks.hpp"


void GetMemory(char *p, int num)
{
	p = static_cast<char*>(malloc(sizeof(char) * num));

	//使用new也能够检测出来
	auto c = new int(100);
}

int main()
{
	DetectMemoryLeaks detectML;
	detectML.SetBreakAlloc(159);
	detectML.SetBreakAlloc(160);
	
	char *str = nullptr;
	GetMemory(str, 100);

	return 0;
}
