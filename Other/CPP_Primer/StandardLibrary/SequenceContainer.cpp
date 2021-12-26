// 通用容器操作 和 顺序容器（SequenceContainer）

#include "pch.h"
#include "SequenceContainer.h"
using namespace std;

// 通用容易操作
void Test_Container()
{
	vector<A>::value_type a4(10);		// value_type：元素类型，A
	vector<A>::reference ra = a4;		// reference：左值引用，A&
	vector<A>::const_reference cra = a4;// const_reference：常左值引用，const A&

	// 构造函数，容器初始化
	vector<A> a2;
	auto iter1 = a2.begin(), iter2 = a2.end();
	vector<A> a5(iter1, iter2);
	vector<A> a6{ 1,2,3,4 };

	// 插入元素
	a5.insert(a5.begin(), 5, A(0));
	cout << a5;
	a5.emplace(a5.begin(), 1);		// 类似于insert，不用构造中间临时变量（C++11）
	cout << a5;

	// 1、容器的拷贝，容器和元素类型必须匹配
	vector<string> v1 = { "asd","qwe" };
	//	vector<const char*> v2 = v1;// err，元素类型
	//	list<string> v3 = v1;		// err，容器类型
		// 2、但传递迭代器范围为参数时，就不要求容器类型匹配
	deque<string> v4(v1.begin(), v1.end());
	cout << "----------Test_Container---------" << endl;
}

// 顺序容器（SequenceContainer）
void Test_string()
{
	string s1 = "abc";
	string s2 = "ABC";
	swap(s1, s2);
	cout << s1 << endl;
	s1.swap(s2);
	cout << s1 << endl;
	cout << s1.length() << endl;
	cout << s1.max_size() << endl;
	cout << "----------Test_string---------" << endl;
}

void Test_vector()
{
	//	std::vector<A> a1(4);		// error，无缺省构造函数
	std::vector<A> a2(4, 12);	// 类型转换构造
	std::vector<A> a3(4, A(12));
	cout << a2;
	cout << a3;

	vector<A>::reverse_iterator ri;

	// 只有顺序容器可以
	vector<A> a7(4);
	vector<A> a8(5, A(12));

	a8.assign({ 1 });
	a8.assign(5, 1);
	cout << a8;

	a8 = { 1,2,3,4,5 };
	cout << a8;

	auto iter1 = a8.begin(); ++iter1;
	auto iter2 = a8.end(); --iter2;
	auto iter = a8.erase(iter1, iter2);
	cout << a8;
	cout << *iter << endl;
	cout << "----------Test_vector---------" << endl;

	vector<int> ivec;
	cout << "ivec: size " << ivec.size() << " capacity: " << ivec.capacity() << endl;

	for (vector<int>::size_type ix = 0; ix != 24; ++ix)
		ivec.emplace_back(ix);
	cout << "ivec: size " << ivec.size() << " capacity: " << ivec.capacity() << endl;

	ivec.reserve(30);
	cout << "ivec: size " << ivec.size() << " capacity: " << ivec.capacity() << endl;

	ivec.insert(ivec.end(), 10, 2);
	cout << "ivec: size " << ivec.size() << " capacity: " << ivec.capacity() << endl;

	ivec.shrink_to_fit();
	cout << "ivec: size " << ivec.size() << " capacity: " << ivec.capacity() << endl;
	cout << "----------Test_vector2---------" << endl;
}

void Test_forward_list()
{
	forward_list<int> a1 = { 1,2,3,4 };
	//	forward_list<int>::reverse_iterator ri;	// 单向链表不支持反向迭代器
	//	cout << a1.size() << endl;				// 不支持
	cout << a1.max_size() << endl;

	// 插入元素
	forward_list<int> a2 = { 1,3,4,5 };
	a2.insert_after(a2.begin(), 2);
	cout << a2;
	a2.erase_after(a2.begin());
	cout << a2;
	a2.erase_after(a2.before_begin());	// 首前元素迭代器
	cout << a2;
	cout << "----------Test_forward_list---------" << endl;
}

void Test_array()
{
	// 数组元素类型和元素个数
	array<int, 4> a1 = { 1,2,3,4 };
	array<int, 4>::iterator iter1 = a1.begin(), iter2 = a1.end();
	//	array<int, 4> a2(iter1, iter2);	// array不支持这种方式

		// 赋值
	a1 = { 2,3,4,5 };

	// 拷贝
	array<int, 4> a3 = a1;

	a3 = { 0,0,0 };
	//	a3.assign(1);	// err，不支持
	cout << a3;
	cout << "----------Test_array---------" << endl;
}