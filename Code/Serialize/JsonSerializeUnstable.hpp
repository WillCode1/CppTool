#pragma once
#include "JsonSerialize.hpp"


class JsonSerializeUnstable :public JsonSerialize<unordered_json>
{
public:

//#define UNSTABLE
#ifndef UNSTABLE
	template <typename Key, typename Value>	bool SetParam(const std::string& strKey, const std::map<Key, Value>& data)
	{
		m_jsonData[strKey] = nlohmann::json(data);

		//std::map<std::string, int> c_map{ {"one", 1}, {"two", 2}, {"three", 3} };
		//m_jsonData = nlohmann::json(c_map);
		return true;
	}
	template <typename Key, typename Value>	bool GetParam(const std::string& strKey, std::map<Key, Value>& data)
	{
		if (!m_jsonData.contains(strKey))
		{
			return false;
		}
		if (!m_jsonData.at(strKey).is_object())
		{
			return false;
		}
		m_jsonData.at(strKey).get_to(data);
		return true;
	}

	/*=====================================================*/
	struct person {
		std::string name;
		std::string address;
		int age;
	};

	/*=====================================================*
	// 创建一个person类并初始化
	person p{ "Ned Flanders", "744 Evergreen Terrace", 60 };

	// 隐式转换: person -> json
	json j = p;

	std::cout << j << std::endl;
	// {"address":"744 Evergreen Terrace","age":60,"name":"Ned Flanders"}

	// 隐式转换: json -> person
	ns::person p2 = j;

	// 测试是否完全相同
	assert(p == p2);

	bool SetParam(const std::string& strKey, const person& data)
	{
		nlohmann::json temp(data);
		person tem = temp.get<person>();
		return true;
	}
	*=====================================================*/

#endif // UNSTABLE
};
