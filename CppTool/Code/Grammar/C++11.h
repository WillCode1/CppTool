#pragma once
#include <iostream>
#include <vector>
#include <algorithm>
#include <memory>
using namespace std;

namespace cpp11 {
	// 引用折叠(Reference Collapsing)/通用引用
	/*
		1)在函数模板隐式推断过程中，若实参为左值，则T&&被推断为左值引用，若实参为右值，则T&&被推断为右值引用，这样的引用被称为通用引用。
		--->就是左值引用会传染，只有纯右值&& && = &&，沾上一个左值引用就变左值引用了
		--->二级引用(指向引用的引用)
		A& & 变成 A&
		A& && 变成 A&
		A&& & 变成 A&
		A&& && 变成 A&&

		2)只有T&&才是通用引用，加了const就不是了。
		3)不做隐式推断，没有通用引用。
		4)类似的规则也同样适用于基于auto关键字的类型推导。
	 */
	class ReferenceCollapse
	{
	public:
		void RunCode(int& a) { cout << "lvalue ref." << endl; }
		void RunCode(const int& a) { cout << "const lvalue ref." << endl; }
		void RunCode(int&& a) { cout << "rvalue ref." << endl; }
		void RunCode(const int&& a) { cout << "const rvalue ref." << endl; }

		// 完美转发
		template<typename T>
		void PerfectForward(T &&t) { RunCode(std::forward<T>(t)); }

		static void testPerfectForward()
		{
			ReferenceCollapse rc;
			int a, b;
			const int c = 0, d = 0;

			rc.PerfectForward(a);
			rc.PerfectForward(move(b));
			rc.PerfectForward(c);
			rc.PerfectForward(move(d));
			std::cout << "============testPerfectForward()===========" << std::endl;
		}
	};

	class NewSkill
	{
	public:
		void TestListInitialization()
		{
			int a{ 4 + 5 };
			int b = { 10 };
			int c(5);
			// 列表初始化会检查类型收窄
			//int d{ 5.0 };	// error

			int* p = new int{ 5 };
			delete p;
		}

		void TestLamdba(int InValue)
		{
			int Value = 0;

			auto a1 = [](int x) {/*仅能访问全局外部变量*/};
			auto a2 = [Value](int x) {/*值传递局部变量Value*/};
			// 默认情况下是一个const函数
			auto a9 = [Value](int x) mutable {/*值传递局部变量Value*/return ++Value; };
			auto a3 = [this](int x) {/*值传递this指针*/};
			auto a4 = [&Value](int x) {/*引用传递局部变量Value*/};
			auto a5 = [=](int x) {/*值传递所有可访问的外部变量*/};
			auto a6 = [&](int x) {/*引用传递所有可访问的外部变量*/};
			auto a7 = [=, &Value](int x) {/*引用传递局部变量Value，值传递所有其他可访问的外部变量*/};
			auto a8 = [&, Value](int x) {/*值传递局部变量Value，引用传递所有其他可访问的外部变量*/};
		}

		template<typename T1, typename T2>
		auto Sum(T1& t1, T2& t2) -> decltype(t1 + t2)
		{
			return t1 + t2;
		}
	};

	// 静态断言，编译时检查
	//static_assert(sizeof(NewSkill) == sizeof(VectorMoveCopy), "sizeof(NewSkill) != sizeof(VectorMoveCopy)!");

	class UnPopular
	{
	public:
		void UseWeakPtr()
		{
			auto sp = make_shared<int>(42);
			weak_ptr<int> wp(sp);
			if (auto spt = wp.lock()) {
				// 只有lock()之后，才有后续操作
			}
		}
	};
};
