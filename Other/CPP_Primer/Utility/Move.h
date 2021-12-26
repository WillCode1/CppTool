#pragma once
#include <cstring>
#include <iostream>
using namespace std;


class String {
public:
	String(char const* str) : m_str(strcpy(
		new char[strlen(str ? str : "") + 1],
		str ? str : "")) {
		cout << "构造函数：" << this << endl;
	}
	~String(void) {
		cout << "析构函数：" << this << endl;
		delete[] m_str;
	}

	String(String const& str) : m_str(strcpy(
		new char[strlen(str.m_str) + 1],
		str.m_str)) {
		cout << "深拷构造：" << &str << "->" << this << endl;
	}
	String(String&& str) : m_str(str.m_str) {
		cout << "移动构造：" << &str << "->" << this << endl;
		str.m_str = nullptr;
	}

	String& operator= (String const& str) {
		cout << "深拷赋值：" << &str << "->" << this << endl;
		if (&str != this) {
			String tmp = str;
			swap(m_str, tmp.m_str);
		}
		return *this;
	}
	String& operator= (String&& str) {
		cout << "移动赋值：" << &str << "->" << this << endl;
		if (&str != this) {
			// 若不转为右值将调用深拷构造而非移动构造
			// 因为右值引用str，也是左值
			String tmp = move(str);
			swap(m_str, tmp.m_str);
		}
		return *this;
	}

	friend ostream& operator<< (ostream& os, String const& str) {
		return os << str.m_str;
	}

private:
	char* m_str;
};

String foo(void);

// 移动语义
void Test_move();