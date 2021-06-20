#pragma once
#include <string>
#include <memory>


class SerializeBase
{
public:
	SerializeBase(const std::wstring& cfg_path) :m_cfgPath(cfg_path) {}
	virtual ~SerializeBase() = default;

	virtual bool Deserialize() = 0;
	virtual bool Serialize() const = 0;

protected:
	std::wstring m_cfgPath;
};

class ConfigBase
{
public:
	virtual ~ConfigBase() = default;

	/// @brief 设置参数
	virtual bool SetParam(const std::string& strKey, const bool& bValue) = 0;
	virtual bool SetParam(const std::string& strKey, const int& nValue) = 0;
	virtual bool SetParam(const std::string& strKey, const double& dValue) = 0;
	virtual bool SetParam(const std::string& strKey, const std::string& strValue) = 0;
	virtual bool SetParam(const std::string& strKey, const std::wstring& strValue) = 0;

	/// @brief 获取参数
	virtual bool GetParam(const std::string& strKey, bool& bValue) const = 0;
	virtual bool GetParam(const std::string& strKey, int& nValue) const = 0;
	virtual bool GetParam(const std::string& strKey, double& dValue) const = 0;
	virtual bool GetParam(const std::string& strKey, std::string& strValue) const = 0;
	virtual bool GetParam(const std::string& strKey, std::wstring& strValue) const = 0;

	virtual void ClearCfg() = 0;
	virtual void ConsolePrint() const = 0;
};
