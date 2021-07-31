//类模版的局部特化
#pragma once
#include <iostream>
using namespace std;

// 完全特化：指定所有类型
// 局部特化：指定部分类型

namespace Template {
	// 通用版本
	template<typename A, typename B>
	class X {
	public:
		static void foo(void) {
			cout << "X<A,B>" << endl;
		}
	};

	// 完全特化
	template<>
	class X<int, short> {
	public:
		static void foo(void) {
			cout << "X<int,short>" << endl;
		}
	};

	// 以下都是局部特化
	template<typename A>
	//B = short
	class X<A, short> {
	public:
		static void foo(void) {
			cout << "X<A,short>" << endl;
		}
	};
	template<typename A>
	//B = A
	class X<A, A> {
	public:
		static void foo(void) {
			cout << "X<A,A>" << endl;
		}
	};
	template<typename A>
	//B = A*
	class X<A, A*> {
	public:
		static void foo(void) {
			cout << "X<A,A*>" << endl;
		}
	};
	template<typename A, typename B>
	//A = void*, B = void*
	class X<A*, B*> {
	public:
		static void foo(void) {
			cout << "X<A*,B*>" << endl;
			cout << typeid(A).name() << ' '
				<< typeid(B).name() << endl;
		}
	};
	template<typename A>
	//A = void*, B = A*
	class X<A*, A*> {
	public:
		static void foo(void) {
			cout << "X<A*,A*>" << endl;
		}
	};

	class TestPartSpec
	{
	public:
		void Test()
		{
			X<int, double>::foo();
			X<int, short>::foo();
			X<double, short>::foo();
			X<int, int>::foo();
			X<double, double*>::foo();
			X<char***, int****>::foo();
			X<double*, double*>::foo();
		}
	};
}
