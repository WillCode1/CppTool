// 测试Json文件读写，以json for modern C++的方式
#include "TestJson.h"
#include <unordered_map>
#include "json.hpp"
#include <fstream>


void TestJson::CreateJson(const std::string& json_path)
{
	nlohmann::json json;
	json["MapInfo"] = nlohmann::json::object();

	std::ofstream ofs(json_path);
	ofs << json;
	ofs.close();
}

bool TestJson::WriteToJson(const std::string& json_path)
{
	std::ifstream ifs(json_path);
	if (ifs.fail())
	{
		CreateJson(json_path);
		ifs.open(json_path);
	}

	nlohmann::json json;

	try
	{
		ifs >> json;
	}
	catch (nlohmann::detail::exception& error)
	{
		ifs.close();
		CreateJson(json_path);
		ifs.open(json_path);
		ifs >> json;
	}
	ifs.close();

	json["Name"] = "Will";
	json["Age"] = "18";
	json["Num"] = "17956455356";

	std::ofstream ofs(json_path);
	ofs << json_path;
	ofs.close();
	return true;
}

void TestJson::JsonBase()
{
	nlohmann::json json;

	{
		//添加一个存储为double的数字
		json["pi"] = 3.141;

		// 添加一个布尔值 
		json["happy"] = true;

		// 添加一个存储为std :: string的字符串
		json["name"] = "Niels";

		// 在对象中添加对象
		json["answer"]["everything"] = 42;

		//添加一个数组，其存储为std::vector（使用初始化列表）
		json["list"] = { 1, 0, 2 };

		// 在一个对象中添加另一个对象
		json["object"] = { {"currency", "USD"}, {"value", 42.99} };

		//创建一个空数组
		json["null_array"] = nlohmann::json::array();
		json["null_array"].emplace_back(1);
		json["null_array"].emplace_back("asd");

		// 创建一个空对象的两种方式
		json["null_obj"] = nlohmann::json({});
		json["null_object"] = nlohmann::json::object();
	}

	std::ofstream ofs("./test.json");
	ofs << json;
	ofs.close();
}
