#include "JsonSerialize.h"
#include <fstream>

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


bool JsonSerialize::SetParam(const std::string & strKey, const bool & bValue)
{
	m_jsonData[strKey] = bValue;
	return true;
}

bool JsonSerialize::SetParam(const std::string & strKey, const int & nValue)
{
	m_jsonData[strKey] = nValue;
	return true;
}

bool JsonSerialize::SetParam(const std::string & strKey, const double & dValue)
{
	m_jsonData[strKey] = dValue;
	return true;
}

bool JsonSerialize::SetParam(const std::string & strKey, const std::string & strValue)
{
	m_jsonData[strKey] = strValue;
	return true;
}

bool JsonSerialize::SetParam(const std::string & strKey, const std::wstring & strValue)
{
	return SetParam(strKey, WStrToStr(strValue));
}

bool JsonSerialize::SetParam(const std::string & strKey, const std::vector<int>& data)
{
	m_jsonData[strKey] = nlohmann::json::array();
	for (auto& i : data)
	{
		m_jsonData[strKey].emplace_back(i);
	}
	return true;
}

bool JsonSerialize::SetParam(const std::string & strKey, const std::vector<double>& data)
{
	m_jsonData[strKey] = nlohmann::json::array();
	for (auto& i : data)
	{
		m_jsonData[strKey].emplace_back(i);
	}
	return true;
}

bool JsonSerialize::SetParam(const std::string& strKey, const std::vector<std::string>& data)
{
	m_jsonData[strKey] = nlohmann::json::array();
	for (auto& i : data)
	{
		m_jsonData[strKey].emplace_back(i);
	}
	return true;
}

bool JsonSerialize::GetParam(const std::string & strKey, bool & bValue) const
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

bool JsonSerialize::GetParam(const std::string & strKey, int & nValue) const
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

bool JsonSerialize::GetParam(const std::string & strKey, double & dValue) const
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

bool JsonSerialize::GetParam(const std::string & strKey, std::string & strValue) const
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

bool JsonSerialize::GetParam(const std::string & strKey, std::wstring & strValue) const
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

bool JsonSerialize::GetParam(const std::string & strKey, std::vector<int>& data)
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

bool JsonSerialize::GetParam(const std::string & strKey, std::vector<double>& data)
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

bool JsonSerialize::GetParam(const std::string & strKey, std::vector<std::string>& data)
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

bool JsonSerialize::LoadCfg()
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

bool JsonSerialize::SaveCfg() const
{
	std::ofstream ofs(m_cfgPath, std::fstream::out);
	if (!ofs.is_open())
	{
		return false;
	}

	ofs << m_jsonData;
	ofs.close();
	return true;
}
