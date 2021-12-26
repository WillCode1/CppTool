// 容器适配器（ContainerAdaptor）
#include "pch.h"
#include "Adaptor.h"
using namespace std;

void Test_Adaptor()
{
	// 使用deque初始化一个栈适配器
	deque<int> deque;
	stack<int> intStack(deque);

	// 基于vector上实现的空栈
	stack<string, vector<string>> str_stack;

	for (size_t ix = 0; ix != 10; ++ix)
		intStack.push(ix);
	while (!intStack.empty())
	{
		cout << intStack.top() << " ";
		intStack.pop();
	}
	cout << "----------Test_Adaptor---------" << endl;
}