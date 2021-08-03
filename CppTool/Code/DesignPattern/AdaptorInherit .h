#pragma once
#include <iostream>
using namespace std;

//双端队列，被适配类
class Deque
{
public:
	void push_back(int x)
	{
		cout << "Deque push_back:" << x << endl;
	}
	void push_front(int x)
	{
		cout << "Deque push_front:" << x << endl;
	}
	void pop_back()
	{
		cout << "Deque pop_back" << endl;
	}
	void pop_front()
	{
		cout << "Deque pop_front" << endl;
	}
};

//顺序类，抽象目标类
class Sequence
{
public:
	virtual void push(int x) = 0;
	virtual void pop() = 0;
};

//栈,后进先出, 适配类
class Stack :public Sequence, private Deque
{
public:
	void push(int x) override
	{
		push_front(x);
	}
	void pop() override
	{
		pop_front();
	}
};

//队列，先进先出，适配类
class Queue :public Sequence, private Deque
{
public:
	void push(int x) override
	{
		push_back(x);
	}
	void pop() override
	{
		pop_front();
	}
};
