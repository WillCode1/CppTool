#pragma once


// 使用VADDR()便捷地获取虚函数地址。 由于功能实现采用的gcc/g++的转换方法，目前应该只支持gcc/g++编译器
class StubTest
{
public:
	virtual int Add(int a, int b)
	{
		return a + b;
	}

	int NoChange(int a)
	{
		a += 100;
		return a;
	}

	bool Pass()
	{
		return false;
	}
};
