#pragma once
#include "JsonSerialize.hpp"


struct Person {
	int a;
	float b;
	double c;
	long d;
	long long e;
	std::string f;
	std::vector<int> g;
	std::map<std::string, double> h;
	std::unordered_map<std::string, int> j;
};

class TestPersonConfig :public JsonSerialize<unordered_json>
{
public:
	TestPersonConfig(const std::wstring& cfg_path) : JsonSerialize(cfg_path) {}

	void SetValue() {
		config.a = 1;
		config.b = 2;
		config.c = 3;
		config.d = 4;
		config.e = 5;
		config.f = "I love you!";

		for (int i = 0; i < 10; ++i) {
			config.g.emplace_back(i);
		}

		for (int i = 0; i < 10; ++i) {
			config.h[std::to_string(i)] = static_cast<double>(i);
		}

		for (int i = 0; i < 10; ++i) {
			config.j[std::to_string(i)] = static_cast<int>(i);
		}
	}

	decltype(m_jsonData)& to_json()
	{
		m_jsonData["a"] = config.a;
		m_jsonData["b"] = config.b;
		m_jsonData["c"] = config.c;
		m_jsonData["d"] = config.d;
		m_jsonData["e"] = config.e;
		m_jsonData["f"] = config.f;
		m_jsonData["g"] = config.g;
		m_jsonData["h"] = config.h;
		m_jsonData["j"] = config.j;

		return m_jsonData;
	}

	void from_json()
	{
		m_jsonData.at("a").get_to(config.a);
		m_jsonData.at("b").get_to(config.b);
		m_jsonData.at("c").get_to(config.c);
		m_jsonData.at("d").get_to(config.d);
		m_jsonData.at("e").get_to(config.e);
		m_jsonData.at("f").get_to(config.f);
		m_jsonData.at("g").get_to(config.g);
		m_jsonData.at("h").get_to(config.h);
		m_jsonData.at("j").get_to(config.j);

		//config.a = m_jsonData.at("a").get<float>();
		//config.b = m_jsonData.at("b").get<float>();
		//config.c = m_jsonData.at("c").get<double>();
		//config.d = m_jsonData.at("d").get<long>();
		//config.e = m_jsonData.at("e").get<long long>();
		//config.f = m_jsonData.at("f").get<std::string>();
		//config.g = m_jsonData.at("g").get<std::vector<int>>();
		//config.h = m_jsonData.at("h").get<std::map<std::string, double>>();
		//config.j = m_jsonData.at("j").get<std::unordered_map<std::string, int>>();
	}

private:
	Person config;
};
