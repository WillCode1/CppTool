#pragma once
#include "ConfigBase.h"
#include <vector>
#include <map>
#include <iostream>
#include <iomanip>
#include <fstream>
#include "json.hpp"
#include "fifo_map.hpp"

// https://cloud.tencent.com/developer/article/1572559
// A workaround to give to use fifo_map as map, we are just ignoring the 'less' compare
template<class K, class V, class dummy_compare, class A>
using fifo_map = nlohmann::fifo_map<K, V, nlohmann::fifo_map_compare<K>, A>;
using unordered_json = nlohmann::basic_json<fifo_map>;

template <class JsonType = nlohmann::json> class JsonSerialize: public ConfigBase, SerializeBase
{
public:
	JsonSerialize(const std::wstring& cfg_path) :ConfigBase(cfg_path)
	{
		LoadCfg();
	}
	~JsonSerialize() override
	{
		SaveCfg();
	}

	/// @brief 设置参数
	bool SetParam(const std::string& strKey, const bool& bValue) override;
	bool SetParam(const std::string& strKey, const int& nValue) override;
	bool SetParam(const std::string& strKey, const double& dValue) override;
	bool SetParam(const std::string& strKey, const std::string& strValue) override;
	bool SetParam(const std::string& strKey, const std::wstring& strValue) override;
	bool SetParam(const std::string& strKey, const std::vector<int>& data);
	bool SetParam(const std::string& strKey, const std::vector<double>& data);
	bool SetParam(const std::string& strKey, const std::vector<std::string>& data);

	/// @brief 获取参数
	bool GetParam(const std::string& strKey, bool& bValue) const override;
	bool GetParam(const std::string& strKey, int& nValue) const override;
	bool GetParam(const std::string& strKey, double& dValue) const override;
	bool GetParam(const std::string& strKey, std::string& strValue) const override;
	bool GetParam(const std::string& strKey, std::wstring& strValue) const override;
	bool GetParam(const std::string& strKey, std::vector<int>& data);
	bool GetParam(const std::string& strKey, std::vector<double>& data);
	bool GetParam(const std::string& strKey, std::vector<std::string>& data);

	bool LoadCfg() override;
	bool SaveCfg() const override;
	void ClearCfg() override;
	void ConsolePrint() const override;

protected:
	JsonType m_jsonData;
};


namespace {
	std::string WStrToStr(const std::wstring& strValue)
	{
		size_t i;
		std::string curLocale = setlocale(LC_ALL, NULL);
		setlocale(LC_ALL, "chs");
		const wchar_t* _source = strValue.c_str();
		size_t _dsize = 2 * strValue.size() + 1;
		char* _dest = new char[_dsize];
		memset(_dest, 0x0, _dsize);
		wcstombs_s(&i, _dest, _dsize, _source, _dsize);
		std::string result = _dest;
		delete[] _dest;
		setlocale(LC_ALL, curLocale.c_str());

		return result;
	}

	std::wstring StrToWStr(const std::string& strValue)
	{
		size_t i;
		std::string curLocale = setlocale(LC_ALL, NULL);
		setlocale(LC_ALL, "chs");
		const char* _source = strValue.c_str();
		size_t _dsize = strValue.size() + 1;
		wchar_t* _dest = new wchar_t[_dsize];
		wmemset(_dest, 0x0, _dsize);
		mbstowcs_s(&i, _dest, _dsize, _source, _dsize);
		std::wstring result = _dest;
		delete[] _dest;
		setlocale(LC_ALL, curLocale.c_str());

		return result;
	}
};


template<class JsonType> bool JsonSerialize<JsonType>::SetParam(const std::string & strKey, const bool & bValue)
{
	m_jsonData[strKey] = bValue;
	return true;
}

template<class JsonType> bool JsonSerialize<JsonType>::SetParam(const std::string & strKey, const int & nValue)
{
	m_jsonData[strKey] = nValue;
	return true;
}

template<class JsonType> bool JsonSerialize<JsonType>::SetParam(const std::string & strKey, const double & dValue)
{
	m_jsonData[strKey] = dValue;
	return true;
}

template<class JsonType> bool JsonSerialize<JsonType>::SetParam(const std::string & strKey, const std::string & strValue)
{
	m_jsonData[strKey] = strValue;
	return true;
}

template<class JsonType> bool JsonSerialize<JsonType>::SetParam(const std::string & strKey, const std::wstring & strValue)
{
	return SetParam(strKey, WStrToStr(strValue));
}

template<class JsonType> bool JsonSerialize<JsonType>::SetParam(const std::string & strKey, const std::vector<int>& data)
{
	m_jsonData[strKey] = nlohmann::json(data);
	return true;
}

template<class JsonType> bool JsonSerialize<JsonType>::SetParam(const std::string & strKey, const std::vector<double>& data)
{
	m_jsonData[strKey] = nlohmann::json(data);
	return true;
}

template<class JsonType> bool JsonSerialize<JsonType>::SetParam(const std::string& strKey, const std::vector<std::string>& data)
{
	m_jsonData[strKey] = nlohmann::json(data);
	return true;
}

template<class JsonType> bool JsonSerialize<JsonType>::GetParam(const std::string & strKey, bool & bValue) const
{
	if (!m_jsonData.contains(strKey))
	{
		return false;
	}
	if (!m_jsonData.at(strKey).is_boolean())
	{
		return false;
	}
	m_jsonData.at(strKey).get_to(bValue);
	return true;
}

template<class JsonType> bool JsonSerialize<JsonType>::GetParam(const std::string & strKey, int & nValue) const
{
	if (!m_jsonData.contains(strKey))
	{
		return false;
	}
	if (!m_jsonData.at(strKey).is_number_integer())
	{
		return false;
	}
	m_jsonData.at(strKey).get_to(nValue);
	return true;
}

template<class JsonType> bool JsonSerialize<JsonType>::GetParam(const std::string & strKey, double & dValue) const
{
	if (!m_jsonData.contains(strKey))
	{
		return false;
	}
	if (!m_jsonData.at(strKey).is_number_float())
	{
		return false;
	}
	m_jsonData.at(strKey).get_to(dValue);
	return true;
}

template<class JsonType> bool JsonSerialize<JsonType>::GetParam(const std::string & strKey, std::string & strValue) const
{
	if (!m_jsonData.contains(strKey))
	{
		return false;
	}
	if (!m_jsonData.at(strKey).is_string())
	{
		return false;
	}
	m_jsonData.at(strKey).get_to(strValue);
	return true;
}

template<class JsonType> bool JsonSerialize<JsonType>::GetParam(const std::string & strKey, std::wstring & strValue) const
{
	if (!m_jsonData.contains(strKey))
	{
		return false;
	}
	if (!m_jsonData.at(strKey).is_string())
	{
		return false;
	}
	strValue = StrToWStr(m_jsonData.at(strKey));
	return true;
}

template<class JsonType> bool JsonSerialize<JsonType>::GetParam(const std::string & strKey, std::vector<int>& data)
{
	if (!m_jsonData.contains(strKey))
	{
		return false;
	}
	if (!m_jsonData.at(strKey).is_array())
	{
		return false;
	}
	m_jsonData.at(strKey).get_to(data);
	return true;
}

template<class JsonType> bool JsonSerialize<JsonType>::GetParam(const std::string & strKey, std::vector<double>& data)
{
	if (!m_jsonData.contains(strKey))
	{
		return false;
	}
	if (!m_jsonData.at(strKey).is_array())
	{
		return false;
	}
	m_jsonData.at(strKey).get_to(data);
	return true;
}

template<class JsonType> bool JsonSerialize<JsonType>::GetParam(const std::string & strKey, std::vector<std::string>& data)
{
	if (!m_jsonData.contains(strKey))
	{
		return false;
	}
	if (!m_jsonData.at(strKey).is_array())
	{
		return false;
	}
	m_jsonData.at(strKey).get_to(data);
	return true;
}

template<class JsonType> bool JsonSerialize<JsonType>::LoadCfg()
{
	std::ifstream ifs(m_cfgPath, std::fstream::in);
	if (!ifs.is_open())
	{
		return false;
	}

	ifs >> m_jsonData;

	ifs.close();
	return true;
}

template<class JsonType> bool JsonSerialize<JsonType>::SaveCfg() const
{
	std::ofstream ofs(m_cfgPath, std::fstream::out);
	if (!ofs.is_open())
	{
		return false;
	}

	ofs << std::setw(4) << m_jsonData << std::endl;
	ofs.close();
	return true;
}

template<class JsonType> void JsonSerialize<JsonType>::ClearCfg()
{
	m_jsonData.clear();
}

template<class JsonType> void JsonSerialize<JsonType>::ConsolePrint() const
{
	std::cout << m_jsonData.dump(4) << std::endl;
}
