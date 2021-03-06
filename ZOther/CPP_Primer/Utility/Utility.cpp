// Utility.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include "pch.h"
#include <iostream>
#include <string>
#include <tuple>
#include "Move.h"
using namespace std;

void Test_lambda()
{
	int z = 80;
//	auto f3 = [=] { cout << ++z << endl; };	// error
	auto f3 = [=] { cout << /*++*/z << endl; };

	// mutable: 在lambda中可以修改按值捕获的外部变量。这样的lambda表达式，其参数表不可省略。
	auto f4 = [=]() mutable { cout << ++z << endl; };
}

//--------------------------------
void Test_tuple()
{
	// tuple可以理解为是对老版本pair类模板的扩展，其中的元素个数不再限于两个，且其功能更加丰富。

	string name = "张飞";
	int    age = 25;
	float  height = 1.75;

	// 直接构造tuple对象
	tuple<string, int, float> st1(name, age, height);
	
	// 获取tuple元素的值
	cout << get<0>(st1) << endl;
	cout << get<1>(st1) << endl;
	cout << get<2>(st1) << endl;
	
	// 通过make_tuple构造tuple对象
	auto st2 = make_tuple(name, age, height);
	cout << get<0>(st2) << endl;
	cout << get<1>(st2) << endl;
	cout << get<2>(st2) << endl;
	get<0>(st2) = "赵云";
	get<1>(st2) = 20;
	get<2>(st2) = 1.65f;
	cout << get<0>(st2) << endl;
	cout << get<1>(st2) << endl;
	cout << get<2>(st2) << endl;
	cout << name << endl;
	cout << age << endl;
	cout << height << endl;
	
	// 按以上两种方式构造的tuple对象所保存的只是构造实参的拷贝
	// 通过tie构造tuple对象，保存构造实参的可写引用
	auto st3 = tie(name, age, height);
	cout << get<0>(st3) << endl;
	cout << get<1>(st3) << endl;
	cout << get<2>(st3) << endl;
	get<0>(st3) = "赵云";
	get<1>(st3) = 20;
	get<2>(st3) = 1.65f;
	cout << get<0>(st3) << endl;
	cout << get<1>(st3) << endl;
	cout << get<2>(st3) << endl;
	cout << name << endl;
	cout << age << endl;
	cout << height << endl;
	
	// 通过tie解析tuple对象
	auto st4 = make_tuple("关羽", 30, 1.85);
	tie(name, age, height) = st4;
	cout << name << endl;
	cout << age << endl;
	cout << height << endl;
	
	// 通过forward_as_tuple构造tuple对象，保存构造实参(本来)引用的对象
	auto st5 = forward_as_tuple(name, age, height);	// 保存左值
	cout << &get<0>(st5) << ' ' << &name << endl;
	cout << &get<1>(st5) << ' ' << &age << endl;
	cout << &get<2>(st5) << ' ' << &height << endl;


	get<0>(st5) = "赵云";
	get<1>(st5) = 20;
	get<2>(st5) = 1.65f;
	cout << name << endl;
	cout << age << endl;
	cout << height << endl;

	auto st6 = forward_as_tuple("关羽", 30, 1.85f);	// 保存右值
	cout << &get<0>(st6) << ' ' << &name << endl;
	cout << &get<1>(st6) << ' ' << &age << endl;
	cout << &get<2>(st6) << ' ' << &height << endl;
	
	// error
	/* 无法对右值引用赋值
	get<0> (st6) = "赵云";
	get<1> (st6) = 20;
	get<2> (st6) = 1.65; */
}

//--------------------------------
void Test_rvalue_reference()
{
	int a;

	// 1)左值引用只能引用左值
	int& lvr1 = a;
//	int& lvr2 = 10; // 错误

	// 2)右值引用只能引用右值
	int&& rvr1 = 10;
//	int&& rvr2 = a; // 错误

	// 3)常左值引用既可以引用左值，也可以引用右值，万能引用
	int const& clvr1 = a;
	int const& clvr2 = 10;
}

template<typename T>
void foo(T&& arg) {
	if (is_lvalue_reference<decltype (arg)>::value)
		cout << "左值引用" << endl;
	else if (is_rvalue_reference<decltype (arg)>::value)
		cout << "右值引用" << endl;
	else
		cout << "不是引用" << endl;
}

template<typename T>
void bar(T const&& arg) {
	if (is_lvalue_reference<decltype (arg)>::value)
		cout << "左值引用" << endl;
	else if (is_rvalue_reference<decltype (arg)>::value)
		cout << "右值引用" << endl;
	else
		cout << "不是引用" << endl;
}

// 通用引用
void Test_universal_reference()
{
	int a = 1, b = 2;
	
	// 通用引用
	foo(a);					// arg:int&
	foo(a + b);				// arg:int&&

	// T const&&只能被推断为常右值引用，无法引用左值a
//	bar(a);			// error
	bar(a + b);

	// 不做隐式推断，没有通用引用
//	foo<int>(a);	// error
	foo<int>(a + b);


	// 类似的规则也同样适用于基于auto关键字的类型推导
	auto&& c = a;	// c:int&
	if (is_lvalue_reference<decltype (c)>::value)
		cout << "左值引用" << endl;
	
	auto&& d = a + b; // d:int&&
	if (is_rvalue_reference<decltype (d)>::value)
		cout << "右值引用" << endl;

	// auto const&&只能被推导为常右值引用，无法引用左值a
//	auto const&& e = a;		// error
	auto const&& f = a + b;
}

template<typename T>
void whatType(void) {
	if (is_lvalue_reference<T>::value)
		cout << "左值引用" << endl;
	else
		if (is_rvalue_reference<T>::value)
			cout << "右值引用" << endl;
		else
			cout << "不是引用" << endl;
}

// 引用折叠
void Test_reference_collapsing_rules()
{
	// 规则：只有右值引用的右值引用才是右值引用，其它情况下的引用折叠一律被视为左值引用
	using lvr_t = int&;
	using rvr_t = int&&;
	whatType<lvr_t>();
	whatType<rvr_t>();
	whatType<lvr_t&>();  // 左值引用的左值引用->左值引用
	whatType<rvr_t&>();  // 右值引用的左值引用->左值引用
	whatType<lvr_t&&>(); // 左值引用的右值引用->左值引用
	whatType<rvr_t&&>(); // 右值引用的右值引用->右值引用
}



int main()
{

//	Test_lambda();
//	Test_tuple();
//	Test_rvalue_reference();
//	Test_universal_reference();
//	Test_reference_collapsing_rules();

	// 移动语义
	Test_move();

	return 0;
}

template<typename T>
void fun(T&& ref) {
	/*
	if (is_lvalue_reference<decltype (ref)>::value)
		foo (ref);
	else if (is_rvalue_reference<decltype (ref)>::value)
		foo (move (ref));
	*/

	// 等价于
	foo(std::forward<T>(ref)); // 完美转发
}

