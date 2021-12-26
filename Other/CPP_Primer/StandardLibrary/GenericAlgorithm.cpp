// GenericAlgorithm(泛型算法)
#include "pch.h"
#include "GenericAlgorithm.h"
//#include <algorithm>
//#include <numeric>

void Test_ReadOnly()
{
	// 使用cbegin(), cend()
	int arr[] = { 1,2,3,4,5,6,7,8,9 };
	std::vector<int> ivec = { 1,2,3,4,5,6,7,8,9 };
	std::vector<string> svec = { "a","b","c" };

	// 查找
	auto ip = std::find(std::cbegin(arr), std::cend(arr), 5);
	auto iter = std::find(ivec.cbegin(), ivec.cend(), 5);
	cout << *ip << endl;
	cout << *iter << endl;

	// 计数
	auto icount = std::count(ivec.cbegin(), ivec.cend(), 5);
	cout << icount << endl;

	// 求和
	auto sum = std::accumulate(ivec.cbegin(), ivec.cend(), 0);
	cout << sum << endl;
	auto sum2 = std::accumulate(svec.cbegin(), svec.cend(), string(""));
	cout << sum2 << endl;

	// 判等
	auto res = std::equal(std::cbegin(arr), std::cend(arr), ivec.cbegin(), ivec.cend());
//	auto res = equal(std::cbegin(arr), std::cend(arr), ivec.cbegin());
	cout << boolalpha << res << endl;

	cout << "----------Test_ReadOnly---------" << endl;
}

void Test_Write()
{
	int arr1[] = { 1,2,3,4,5 };
	int arr2[sizeof(arr1)/sizeof(*arr1)];
	vector<int> ivec1 = { 1,2,3,4,5 };
	vector<int> ivec2;
	vector<int> ivec3;

	// 拷贝
	std::copy(std::cbegin(arr1), std::cend(arr1), std::begin(arr2));
	for (auto i : arr2) cout << i << " "; cout << endl;
	std::copy(ivec1.begin(), ivec1.end(), std::back_inserter(ivec2));
	cout << ivec2;

	// 代替
	std::replace(ivec1.begin(), ivec1.end(), 1, 2);
	cout << ivec1;
	std::replace_copy(std::cbegin(arr1), std::cend(arr1), std::back_inserter(ivec3), 1, 2);
	cout << ivec3;

	// 排序、排重
	ivec2 = { 4,2,3,3,3,2,4,1,2,3,4 };
	std::sort(ivec2.begin(), ivec2.end());
	auto unique_end = std::unique(ivec2.begin(), ivec2.end());
	ivec2.erase(unique_end, ivec2.end());
	cout << ivec2;

	cout << "----------Test_Write---------" << endl;
}

void Test_Lambda()
{
	int a = 10, b = 20;
	std::vector<int> ivec = { 1,2,3,4,12,16 };
	std::vector<int> ivec2(ivec.size(), 0);

	auto foo = [] ()->int { return 24; };
	cout << foo() << endl;

	auto res = std::find_if(ivec.cbegin(), ivec.cend(), [a, b](const int num) { return num > b - a; });
	cout << *res << endl;

//	auto f = [&a]() { return ++a; };		// 引用捕获
	auto f = [a]() mutable { return ++a; };	// 可变lambda
	cout << f() << endl;

	std::transform(ivec.cbegin(), ivec.cend(), ivec2.begin(),
		[](int i) ->int { if (i > 10) return i - 10; else return i; });
	cout << ivec2;

	cout << "----------Test_Lambda---------" << endl;
}

// list 和 forward_list 独有操作和通用版本对比
void Test_List()
{
	std::vector<int> ivec = { 2,5,3,7,8 };
	std::vector<int> ivec2(10,0);
	std::list<int> ilist = { 2,5,3,7,8 };
	std::list<int> ilist2;
	
	{	// sort
		std::sort(ivec.begin(), ivec.end());
		cout << ivec;

		ilist.sort();
		cout << ilist;
	}
	{	// merge（合并，需要两条序列有序）
		std::merge(ivec.begin(), ivec.end(), ilist.begin(), ilist.end(), ivec2.begin());
		cout << ivec2;

		ilist2.merge(ilist);
		cout << ilist2;
	}
	{	// remove_if
		auto remove_iter = std::remove_if(ivec.begin(), ivec.end(), [](const int a) { return a < 4; });
		ivec.erase(remove_iter, ivec.end());
		cout << ivec;

		ilist2.remove_if([](const int a) { return a < 4; });
		cout << ilist2;
	}
	{	// reverse（反转顺序）
		std::reverse(ivec.begin(), ivec.end());
		cout << ivec;

		ilist2.reverse();
		cout << ilist2;
	}
	{	// splice（剪切，链表类型独有，没有通用版本）
		list<int> list1 = { 1,2,3,7 };
		list<int> list2 = { 4,5,6 };
		list<int> list3 = { 8,9 };

		list1.splice(list1.end(), list3);
		cout << list1;
		auto iter = ++++++list1.begin();
		list1.splice(iter++, list2, list2.begin());
		cout << list1;
		list1.splice(iter, list2, list2.begin(), list2.end());
		cout << list1;
	}
	cout << "----------Test_List---------" << endl;
}