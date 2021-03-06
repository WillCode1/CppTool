// SmartPointer.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
// 智能指针

#include "pch.h"
#include <iostream>
#include <memory>
#include <string>
#include <list>
using namespace std;

class A
{
public:
	A() = default;
	A(int _a) : a(_a) {};
	~A() = default;

public:
	int a;
};


// shared_ptr
namespace sp {
	// shared_ptr独有的操作
	void Test1()
	{
		// 1、初始化
		auto p1 = std::make_shared<int>(10);
		auto p2 = std::make_shared<int>(100);

		// 2、拷贝构造
		// p1的引用计数++
		std::shared_ptr<int> p3(p1);

		// 3、拷贝赋值
		p3 = p2;	// p3的引用计数--，p2的引用计数++
		p2 = p1;	// p2的引用计数--，p1的引用计数++

		// 4、判断共享对象的引用是否唯一
		cout << boolalpha;
		cout << p1.unique() << endl;	// false
		cout << p3.unique() << endl;	// true

		// 5、获得共享对象的引用计数
		cout << p1.use_count() << endl;	// 2
	}

	// reset的使用
	void Test2()
	{
		std::shared_ptr<int> sp;
		sp.reset(new int(10));

		// 使用unique和reset，对共享对象深拷贝
		if (!sp.unique())
			sp.reset(new int(20));
		*sp += 10;
	}
}

// unique_ptr
namespace up {
	void Test1()
	{
		// 1、初始化
		std::unique_ptr<string> p1(new string("p1"));
		cout << *p1 << endl;
		// 2、转移控制权
		std::unique_ptr<string> p2(p1.release());
		cout << *p2 << endl;
		// 3、释放p2内存，转移p3给p2
		std::unique_ptr<string> p3(new string("p3"));
		p2.reset(p3.release());
		cout << *p2 << endl;
	}
}

namespace {
	void Test()
	{
		// 智能指针默认初始化为nullptr
		std::shared_ptr<std::string> p1;
		std::shared_ptr<std::string> p2;

		// 检查是否指向空string
		if (p1 && p1->empty())
			*p1 = "hi";

		cout << p1.get() << endl;

		swap(p1, p2);
		p1.swap(p2);
	}
	
	void Test_New()
	{
		// 1、new一个const对象
		const int *pci = new const int(1024);
		delete pci;

		// 2、设定内存分配失败，返回nullptr
		int *p1 = new int;				// 如分配失败，new抛出std::bad_alloc
		int *p2 = new (nothrow) int;	// 如分配失败，返回nullptr
		delete p1, p2;
	}

	// 不要和普通指针混用
	void Test2()
	{
		int *p = new int(1024);
		{
			auto sp = std::shared_ptr<int>(p);	// 合法，但是p被释放
		}
		int a = *p;			// p悬空
		cout << a << endl;	// 未定义
	}
}

int main()
{
//	::Test();
//	::Test2();

//	sp::Test1();
//	sp::Test2();

//	up::Test1();

	std::list<shared_ptr<int>> intlist;
//	intlist.push_back(new int(10));		// error
	intlist.emplace_back(new int(10));

	return 0;
}
