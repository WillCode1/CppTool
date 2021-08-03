#pragma once
#include <iostream>


// gmock 依赖C++多态机制进行工作，只有虚函数才能被mock
class ExtendedKalmanFilter
{
public:
	//virtual int Add(int a, int b) = 0;
	virtual int Add(int a, int b)
	{
		return a + b;
	}

	virtual int NoChange(int a)
	{
		return a;
	}

	virtual bool Pass()
	{
		return false;
	}

	void Print()
	{
		std::cout << "Print......" << std::endl;
	}
};
