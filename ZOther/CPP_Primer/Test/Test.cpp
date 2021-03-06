// Test.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include "pch.h"
#include <iostream>
#include <string>
#include <unordered_map>
#include "json.hpp"
using namespace std;
void TestJson();
void FuzzyEnquiry(const std::list<std::string>& str_list, const std::string& key1, const std::string& key2, const std::string& key3);

class A {
	virtual void foo1() {}
	virtual void foo2() {}
	void foo3() {}

	//char a;
};

class B :public A
{

};

int main()
{
	cout << sizeof(A) << endl;
	cout << sizeof(B) << endl;

	TestJson();

	{
		int num;
		string num_str = "标高 14";
		auto p = num_str.c_str();
		while ((*p <= '0'|| *p >= '9') && *p != '\0')
		{
			p++;
		}
		if (*p != '\0')
			num = std::atoi(p);


	}

	return 0;
}

void TestJson()
{
	using Info = std::unordered_map<std::string, std::string>;

	nlohmann::json json;

	Info info_map;
	info_map.emplace("age", "18");
	info_map.emplace("name", "asd");

	std::unordered_map<std::string, std::list<Info>> json_map;
	json_map["default"].emplace_back(info_map);
	json_map["default"].emplace_back(info_map);

	json = json_map;

	cout << json << endl;
	cout << json_map.begin()->first << endl;
}

void FuzzyEnquiry(const std::list<std::string>& str_list, const std::string& key1, const std::string& key2, const std::string& key3)
{
	for (const auto& str_item : str_list)
	{
		if ((str_item.find(key1) != string::npos) || (str_item.find(key2) != string::npos) || (str_item.find(key3) != string::npos))
		{
			cout << str_item << endl;
		}
	}
}
