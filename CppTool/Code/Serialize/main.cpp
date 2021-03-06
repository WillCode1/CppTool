// Serialize.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include "TestJson.h"
#include "JsonSerialize.hpp"
#include "TestPersonConfig.h"


int main()
{
	//TestJson::JsonBase();
	//TestJson::JsonFifo();

	JsonSerialize<unordered_json> js(L"./test2.json");

	int age = 18;
	std::wstring name = L"Will";
	std::vector<std::string> names = { "Will", "Nance" };
	std::map<int, std::string> index_name = { {1, "Will"} };

	js.SetParam("name", name);
	js.SetParam("dge", 133);
	js.SetParam("age", age);
	js.SetParam("list", names);
	//js.SetParam("table", index_name);
	index_name.clear();

	js.Serialize();
	js.ConsolePrint();
	js.Deserialize();
	js.GetParam("age", age);
	js.GetParam("name", name);
	js.GetParam("list", names);
	//js.GetParam("table", index_name);

	TestPersonConfig tp(L"./test3.json");
	tp.SetValue();
	tp.to_json();
	tp.from_json();

	return 0;
}
