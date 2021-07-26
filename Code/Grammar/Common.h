#pragma once
#include <vector>
#include <algorithm>
#include <memory>
using namespace std;

class A
{
	void foo() {}
};

class B :public A
{
public:
	virtual void foo() {}

	// “inline”是构造函数的唯一合法存储类
	/*virtual*/ B() {}

private:
	// “B::~B”: 无法访问 private 成员(在“B”类中声明)
	//~B() {}
};

class Common
{
public:
	void VectorUnique()
	{
		vector<int> ivec = { 4,2,3,3,3,2,4,1,2,3,4 };
		std::sort(ivec.begin(), ivec.end());
		auto unique_end = std::unique(ivec.begin(), ivec.end());
		ivec.erase(unique_end, ivec.end());
	}
};

