#pragma once
#include "ConfigBase.h"
#include <vector>
#include <map>
#include "json.hpp"
#include "fifo_map.hpp"

#define FIFO_JSON


class JsonSerialize: public ConfigBase, SerializeBase
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

private:
	// https://cloud.tencent.com/developer/article/1572559
	// A workaround to give to use fifo_map as map, we are just ignoring the 'less' compare
	template<class K, class V, class dummy_compare, class A>
	using fifo_map = nlohmann::fifo_map<K, V, nlohmann::fifo_map_compare<K>, A>;
	using unordered_json = nlohmann::basic_json<fifo_map>;

#ifdef FIFO_JSON
	unordered_json m_jsonData;
#else
	nlohmann::json m_jsonData;
#endif
};
