//类模版的特化
#pragma once
#include <cstring>
#include <iostream>
using namespace std;

// 全类特化：特化类模板所有成员函数
// 成员特化：针对部分成员函数进行特化

namespace Template {
	// 通用版本
	template<typename T>
	class Comparator {
	public:
		Comparator(T x, T y) :m_x(x), m_y(y) {}
		T max(void) const
		{
			return m_x < m_y ? m_y : m_x;
		}
	private:
		T m_x, m_y;
	};

#if 0
	// 全类特化
	template<>
	class Comparator<char*>
	{
	public:
		Comparator(char* x, char* y) :m_x(x), m_y(y) {}
		char* max(void) const
		{
			return strcmp(m_x, m_y) < 0 ? m_y : m_x;
		}
	private:
		char* m_x, *m_y;
	};
#else
	// 成员特化
	template<>
	char* Comparator<char*>::max(void) const
	{
		return strcmp(m_x, m_y) < 0 ? m_y : m_x;
	}
#endif

	class TestSpec
	{
	public:
		void Test()
		{
			int a = 123, b = 345;
			char c[] = "hello", d[] = "world";

			Comparator<int> ci(a, b);
			cout << ci.max() << endl;
			Comparator<char*> cs(c, d);
			cout << cs.max() << endl;
		}
	};
}
