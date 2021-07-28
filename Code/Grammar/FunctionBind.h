#pragma once
#include <iostream>
#include <map>
#include <functional>
using namespace std;
using namespace std::placeholders;

// https://zhuanlan.zhihu.com/p/381639427

namespace cpp11 {
	//有这样的重载函数
	int good(int) { return 0; }
	double good(double) { return 0.0; }

	int funcValue(int x)
	{
		return x;
	}

	class Functor
	{
	public:
		int operator()(int a)
		{
			return a;
		}
	};

	class CFunc
	{
	public:
		int Func(int a)
		{
			return a;
		}
		static int staticFunc(int a)
		{
			return a;
		}
	};

	class FunctionAndBind
	{
	public:
		void TestBind()
		{
			auto f = [](int a, char, double)->int { return a; };
			auto ff = std::bind(f, _1, 'c', 2.2);
			int result = ff(10);     // f(10,'c',2.2)

			auto reflect = std::bind(f, _3, _2, _1);    // 翻转参数顺序
			result = reflect(2.2, 'c', 10);				// f(10,'c',2.2); 


			//auto result = std::bind(good, _1);      //错误形式，不知道调用哪一个good函数
			//正确的做法，但是比较复杂
			auto result_1 = std::bind((double(*)(double))good, _1);
			//这是更好的方式
			//auto result_2 = std::bind<double>(good, _1);   //指定函数的返回类型
		}

		void TestFunction()
		{
			using Functional = function<int(int)>;

			//封装普通函数
			Functional obj = funcValue;
			int result = obj(100);

			//封装lambda表达式
			obj = [](int a)->int { return a; };
			result = obj(100);

			//封装仿函数
			Functor funObj;
			obj = funObj;
			result = obj(100);

			//封装类的成员函数
			CFunc func;
			obj = std::bind(&CFunc::Func, &func, std::placeholders::_1);
			result = obj(100);

			//封装类的静态函数
			obj = CFunc::staticFunc;
			result = obj(100);
		}
	};
};
