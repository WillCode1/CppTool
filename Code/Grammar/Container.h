#pragma once
#include <string>
#include <iostream>
#include <vector>
#include <list>
#include <map>           // 基于红黑树的有序映射
#include <unordered_map> // 基于散列表的无序映射
using namespace std;

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


namespace cpp11 {
	class Container
	{
	public:
		void TestList()
		{

		}

		void TestUnorderedMap()
		{
			map<string, int> m1{ {"zhangfei", 80}, {"guanyu", 90}, {"zhaoyun", 60} };
			for (auto a : m1)
				cout << a.first << "：" << a.second << endl;
			cout << "--------" << endl;

			unordered_map<string, int> m2{ {"zhangfei", 80}, {"guanyu", 90}, {"zhaoyun", 60} };
			for (auto a : m2)
				cout << a.first << "：" << a.second << endl;
			cout << "--------" << endl;
			
			unordered_map<Student, int, Hash, Equal> m3{ {{"zhangfei", 20}, 80}, { {"guanyu", 30}, 90}, { {"zhaoyun", 25}, 60} };
			for (auto a : m3)
				cout << a.first << "：" << a.second << endl;
		}
	};
}
