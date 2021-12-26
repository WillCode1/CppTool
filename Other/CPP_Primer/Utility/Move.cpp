#include "pch.h"
#include "Move.h"
#include <iostream>
using namespace std;


String foo(void) {
	String str("Hello, World !");
	return str;
}

// 移动语义
void Test_move()
{
	String a("Hello, C++11 !"), b("Hello, C++14 !");
	cout << "-------- 1 --------" << endl;
	String c = a; // 深拷构造
	cout << a << endl;
	cout << c << endl;
	cout << "-------- 2 --------" << endl;
	c = b; // 深拷赋值
	cout << b << endl;
	cout << c << endl;
	cout << "-------- 3 --------" << endl;
	String d = foo(); // 移动构造
	cout << d << endl;
	cout << "-------- 4 --------" << endl;
	c = foo(); // 移动赋值
	cout << c << endl;
	cout << "-------- 5 --------" << endl;
	String e = move(a); // 移动构造
	cout << e << endl;
	cout << "-------- 6 --------" << endl;
	c = move(b); // 移动赋值
	cout << c << endl;
	cout << "-------- 7 --------" << endl;
}

