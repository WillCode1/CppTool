#pragma once
#include <iostream>
#include <vector>
#include <list>
#include <deque>

using namespace std;

class Base
{
public:
	// 委托构造函数
	Base() :Base(0) {}
	Base(int _b) :Base(_b, 0.0) {}
	Base(double _c) :Base(0, _c) {}
	Base(int _b, double _c) :b(_b), c(_c) { init(); }

	// 构造函数泛型编程
	Base(vector<short>& vec) : l(vec.begin(), vec.end()) {}
	Base(deque<int>& deq) : l(deq.begin(), deq.end()) {}
	template<class T> Base(T first, T last) :l(first, last) {}

	virtual ~Base() {}

private:
	void init() {}

protected:
	int b;
	double c;

	list<int> l;
};

class Derived :public Base
{
public:
	using Base::Base;	// 继承构造函数
	virtual ~Derived() {}

private:
	int d{ 100 };
};

