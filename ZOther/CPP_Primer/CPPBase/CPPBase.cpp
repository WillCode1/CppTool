// C++ 11 中新语法演示

#include "pch.h"
#include <iostream>
#include <string>
#include <vector>

#define NDEBUG
#include <cassert>
using namespace std;

class MyClass
{
public:
	MyClass() = default;	// 编译器提供的缺省构造
	~MyClass() = default;
	MyClass(int a) :m_a(a) {}

public:
	int m_a = 0;
//	MyClass m_c;			// error，不完整类型
	static int size;
	static MyClass m_c;		// 静态成员可以是不完整类型
};
int MyClass::size = 10;

vector<MyClass> foo()
{
	if (true)
		return { (1),(2),(3) };
}

// auto
void Test_Auto()
{
	// 1.声明多个变量时，基本数据类型必须一致
//	auto x = 100, y = 3.14;		// error

	// 2.auto忽略顶层const（修饰对象本身的const）
	int i = 10;
	const int ci = i, &cr = ci;	// (顶层const)，int
	auto b = ci, c = cr;		// int
	b++; c++;
	const auto d = b;			// const int
//	d++;	// error

	// 3.auto的引用保留顶层const
	auto &m = ci, *p = &ci;		// const int
//	*p++;	// error;
}

// decltype简单介绍：类型推断
void Test_Decltype()
{
	std::string str = "asd";
	decltype(str.size()) a = 10;// 推断函数返回值类型
	int b = 10, *p = &b;
	decltype(a + b) c = -20;	// 推断表达式类型
	std::cout << c << std::endl;
	decltype((10)) f;			// 括号只针对常量
	decltype((str)) d = str;	// 括号表达式为左值，引用
	decltype(*p) e = b;			// 表达式为左值，引用
}

// 范围for（range for）
void Test_RangeFor()
{
	// 遍历数组中每一个元素
	int arr[10] = { 0,1,2,3,4,5,6,7,8,9 };
	for (auto i : arr)
		std::cout << i;
	std::cout << std::endl;
	// 遍历str中每一个字符
	std::string str("hello world!");
	for (auto& c : str)
		c = toupper(c);
	std::cout << str << std::endl;
}

// vector对象的初始化方式
void Test_Vector()
{
	std::vector<int> v1(3, 1);		// 1,1,1
	std::vector<int> v2(3);			// 0,0,0
	std::vector<int> v3{ 1,2,3 };	// 1,2,3
	std::vector<int> v4 = { 1,2,3 };// 1,2,3

	// 数组int_arr中的一部分，初始化vector
	int int_arr[4] = { 1,2,3,4 };
	// 使用std::begin，std::end计算数组首尾地址
	std::vector<int> v5(std::begin(int_arr) + 1, std::end(int_arr));
	for (auto i : v5)
		std::cout << i;
	std::cout << std::endl;
}

// 简洁是一种美德
void Concise()
{
	int array[10] = { 1,2,3 };
	int *iter = array;
	cout << *iter++ << endl;	// 1
	// 等价于
	cout << *iter << endl;		// 2
	++iter;
}

// 列表初始化返回值
void Test_Init()
{
	vector<MyClass> myvector = foo();
	for (auto i : myvector)
	{
		cout << i.m_a << endl;
	}
}

int main()
{
//	Test_Auto();
//	Test_Decltype();
//	Test_RangeFor();
	Test_Vector();

	assert(0);

	cin.get();
	return 0;
}

