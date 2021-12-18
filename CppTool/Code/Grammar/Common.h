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

class C
{
public:
    A& a;
};

class Common
{
public:
	void TestTypeidAndDecltype()
	{
		cout << typeid(A).name() << endl;
		cout << typeid(B).name() << endl;

		cout << typeid(A).hash_code() << endl;
		cout << typeid(B).hash_code() << endl;
	}
};

