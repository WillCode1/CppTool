#pragma once
#include <iostream>
#include <vector>
#include <list>
#include <deque>

using namespace std;


namespace cpp11 {
	// 构造函数泛型编程
	class TemplateConstruct
	{
	public:
		TemplateConstruct(vector<short>& vec) : l(vec.begin(), vec.end()) {}
		TemplateConstruct(deque<int>& deq) : l(deq.begin(), deq.end()) {}
		template<class T> TemplateConstruct(T first, T last) : l(first, last) {}

	private:
		list<int> l;
	};

	/*
		1.委托构造函数
		2.深拷贝
		3.移动拷贝构造/赋值
		4.右值引用
	 */
	class Base
	{
	public:
		// 委托构造函数
		Base() :Base(0.0) {}
		Base(int _b) :Base(_b, 0.0) {}
		// 目标构造函数
		Base(double _c) :p(nullptr), c(_c) { init(); }
		Base(int _b, double _c) :p(new int(_b)), c(_c) { init(); }

		virtual ~Base() { delete p; cout << this << " Deconstruct." << endl; }

		// 拷贝构造/赋值
		Base(const Base& other)
		{
			if (this == &other)
			{
				return;
			}
			if (other.p)
			{
				p = new int(*other.p);
			}
			else
			{
				p = nullptr;
			}
			c = other.c;
			cout << this << " Copy Construct from " << &other << "." << endl;
		}
		Base& operator=(const Base& other)
		{
			if (this == &other)
			{
				return *this;
			}
			if (p)
			{
				this->~Base();
				//cout << "Leak." << endl;
			}
			if (other.p)
			{
				p = new int(*other.p);
			}
			else
			{
				p = nullptr;
			}
			c = other.c;
			cout << this << " Copy Assign from " << &other << "." << endl;
			return *this;
		}

		// 移动拷贝构造/赋值
		Base(Base&& other)
		{
			if (this == &other)
			{
				return;
			}
			p = other.p;
			c = other.c;
			other.p = nullptr;
			cout << this << " Move Copy Construct from " << &other << "." << endl;
		}
		Base& operator=(Base&& other)
		{
			if (this == &other)
			{
				return *this;
			}
			if (p)
			{
				this->~Base();
				//cout << "Leak." << endl;
			}
			p = other.p;
			c = other.c;
			other.p = nullptr;
			cout << this << " Move Copy Assign from " << &other << "." << endl;
			return *this;
		}

		static Base ReturnValue()
		{
			Base temp;
			return temp;
		}

#if 0
		// 禁止返回局部右值引用
		static Base&& ReturnValue(int)
		{
			return Base();
		}
#endif // 0
		void setC(const double& _c)
		{
			c = _c;
		}

	private:
		void init() { cout << this << " Construct." << endl; }

	protected:
		int* p;
		double c;
	};


	class Derived :public Base
	{
	public:
		using Base::Base;	// 继承构造函数
		virtual ~Derived() {}

	private:
		int d{ 100 };
	};

	/* STL vector move copy */
	class VectorMoveCopy
	{
	public:
		VectorMoveCopy(const std::vector<int>& vec)
		{
			int_vec = vec;
		}
		VectorMoveCopy(const VectorMoveCopy& other) = default;
		VectorMoveCopy(VectorMoveCopy&& other)
		{
			if (&other == this)
			{
				return;
			}

			int_vec = std::move(other.int_vec);
			std::cout << "Move Copy" << std::endl;
		}
		static void testVectorMoveCopy()
		{
			B test;
			std::cout << sizeof(A) << std::endl;
			std::cout << sizeof(B) << std::endl;

			std::vector<int> vec = { 1,2,3 };
			cpp11::VectorMoveCopy a(vec);
			cpp11::VectorMoveCopy b = std::move(a);
			std::cout << "============testVectorMoveCopy()===========" << std::endl;
		}
	private:
		std::vector<int> int_vec;
	};
}
