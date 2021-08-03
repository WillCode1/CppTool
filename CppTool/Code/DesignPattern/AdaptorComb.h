#pragma once
#include <iostream>
using namespace std;

/*
* 关键代码：适配器继承或依赖已有的对象，实现想要的目标接口。
* 以下示例中，假设我们之前有了一个双端队列，新的需求要求使用栈和队列来完成。
  双端队列可以在头尾删减或增加元素。而栈是一种先进后出的数据结构，添加数据时添加到栈的顶部，删除数据时先删   除栈顶部的数据。因此我们完全可以将一个现有的双端队列适配成一个栈。
*/

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
class Stack :public Sequence
{
public:
	void push(int x) override
	{
		m_deque.push_front(x);
	}
	void pop() override
	{
		m_deque.pop_front();
	}
private:
	Deque m_deque;
};

//队列，先进先出，适配类
class Queue :public Sequence
{
public:
	void push(int x) override
	{
		m_deque.push_back(x);
	}
	void pop() override
	{
		m_deque.pop_front();
	}
private:
	Deque m_deque;
};
