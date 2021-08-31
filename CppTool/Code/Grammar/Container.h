#pragma once
#include <string>
#include <iostream>
#include <vector>
#include <list>
#include <forward_list>
#include <map>           // 基于红黑树的有序映射
#include <unordered_map> // 基于散列表的无序映射
#include <algorithm>
using namespace std;

// https://zh.cppreference.com/w/cpp/container
namespace cpp11 {
	class Student {
	public:
		Student(string const& name = "", int age = 0) :
			m_name(name), m_age(age) {}
		friend ostream& operator<< (ostream& os, Student const& s) {
			return os << s.m_name << ',' << s.m_age;
		}
	private:
		string m_name;
		int m_age;
		friend class Hash;
		friend class Equal;
	};

	// 哈希器
	class Hash {
	public:
		size_t operator() (Student const& s) const {
			return std::hash<string>() (s.m_name);
		}
	};

	// 判等器
	class Equal {
	public:
		bool operator() (Student const& a, Student const& b) const {
			return a.m_name == b.m_name && a.m_age == b.m_age;
		}
	};

	class Container
	{
	public:
		void TestVector()
		{
			std::vector<int> vec = { 4,2,1,3 };
			std::sort(vec.begin(), vec.end());
			auto res = vec.at(3);

			// 只改变size
			vec.resize(6, 1);

			// 设置预留空间大小
			vec.reserve(100);
			auto temp = vec.capacity();
			
			// capacity = size
			vec.shrink_to_fit();
			temp = vec.capacity();
			// 获取向量的最大大小，它返回向量可以存储的元素总数
			temp = vec.max_size();

			vector<int> ivec = { 4,2,3,3,3,2,4,1,2,3,4 };
			std::sort(ivec.begin(), ivec.end());
			auto unique_end = std::unique(ivec.begin(), ivec.end());
			ivec.erase(unique_end, ivec.end());
		}

		void TestList()
		{
			//std::forward_list<int> lst = { 4,2,1,3 };
			std::list<int> lst = { 4,2,1,3 };
			std::list<int> lst2 = { 40,20,10,30 };

			auto iter = lst.begin();
			std::advance(iter, 2);
			lst.splice(iter, lst2);

			lst2.splice(lst2.begin(), lst, iter, lst.end());

			lst.sort();
			lst2.sort();
			// 归并二个已排序链表为一个。链表应以升序排序
			lst.merge(lst2);

			lst.reverse();
			lst.sort();
			lst.unique();
			lst.remove_if([](int n) { return n > 10; });
		}

		void TestUnorderedMap()
		{
			map<string, int> m1{ {"zhangfei", 80}, {"guanyu", 90}, {"zhaoyun", 60} };
			for (auto a : m1)
				cout << a.first << ": " << a.second << endl;
			cout << "--------" << endl;

			unordered_map<string, int> m2{ {"zhangfei", 80}, {"guanyu", 90}, {"zhaoyun", 60} };
			for (auto a : m2)
				cout << a.first << ": " << a.second << endl;
			cout << "--------" << endl;
			
			unordered_map<Student, int, Hash, Equal> m3{ {{"zhangfei", 20}, 80}, { {"guanyu", 30}, 90}, { {"zhaoyun", 25}, 60} };
			for (auto a : m3)
				cout << a.first << ": " << a.second << endl;
		}

		void TestMultimap()
		{
			multimap<string, int> msi;
			msi.insert(make_pair("zhangfei", 80));
			msi.insert(make_pair("zhaoyun", 70));
			msi.insert(make_pair("guanyu", 60));
			msi.insert(make_pair("zhangfei", 50));
			msi.insert(make_pair("zhaoyun", 40));
			msi.insert(make_pair("guanyu", 30));
			for (auto it = msi.begin(); it != msi.end(); ++it)
				cout << it->first << ": " << it->second << endl;
			cout << "=============================" << endl;

			typedef multimap<string, int>::iterator IT;
			//返回符合映射内容的迭代器范围,保存在pair对象中
			pair<IT, IT> res = msi.equal_range("zhangfei");
			int sum = 0;
			for (auto it = res.first; it != res.second; ++it)
				sum += it->second;
			cout << "zhangfei: " << sum << endl;
		}

		// c++优先队列(priority_queue)用法详解：https://www.cnblogs.com/huashanqingzhu/p/11040390.html
	};
}
