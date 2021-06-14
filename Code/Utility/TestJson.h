// 测试Json文件读写，以json for modern C++的方式
#pragma once
#include <string>

class TestJson
{
public:
	TestJson() = default;
	~TestJson() = default;

	static void CreateJson(const std::string& json_path);
	static bool WriteToJson(const std::string& json_path);

	static void JsonBase();
};

