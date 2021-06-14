// 字符串转换工具
#pragma once
#include <string>
#include <vector>

#ifdef COMMON_MODULE
#define EXPORTCOMMONCLASS __declspec(dllexport)
#define EXPORTCOMMONAPI __declspec(dllexport)
#else
#define EXPORTCOMMONCLASS __declspec(dllimport)
#define EXPORTCOMMONAPI __declspec(dllimport)
#endif


class EXPORTCOMMONCLASS StringTool
{
public:
	static std::string Wchar_tToString(const wchar_t* wchar);

	/// @brief string转wstring
	static std::wstring StrToWStr(const std::string& strValue);
	/// @brief wstring转string
	static std::string WStrToStr(const std::wstring& strValue);
	/// @brief  UTF8转Unicode
	static std::wstring UTF8ToUnicode(const std::string& str);
	/// @brief  Unicode转UTF8
	static std::string UnicodeToUTF8(const std::wstring& str);
	/// @brief  ANSI转Unicode
	static std::wstring ANSIToUnicode(const std::string& str);
	/// @brief  Unicode转ANSI
	static std::string UnicodeToANSI(const std::wstring& str);
	/// @brief UTF8转GBK
	static std::string UTF8ToGBK(const std::string& strUtf8);
	/// @brief GBK转UTF8
	static std::string GBKToUTF8(const std::string& strGBK);

	/// @brief wstring转成double
	static int StrToInt(const std::wstring& strValue);
	/// @brief wstring转成double
	static double StrToDouble(const std::wstring& strValue);
	/// @brief int转成wstring
	static std::wstring IntToStr(int nValue, int nRadix = 10);
	/// @brief double转成wstring
	static std::wstring DoubleToStr(const double& dValue, int accuracy = 8);

	/// @brief 分割字符串
	static bool SplitStr(const std::wstring& str, const std::wstring& tag, std::vector<std::wstring>& vec);
	/// @brief 字符串strValue是否全为strInclude字符组合
	static bool SpanIncluding(const std::wstring& strValue, const std::wstring& strInclude);

	/// @brief 转小写
	static std::wstring ToLower(const std::wstring& strValue);
	/// @brief 转大写
	static std::wstring ToUpper(const std::wstring& strValue);

	/// @brief 是否数字
	static bool IsDigit(const wchar_t& ch);
	static bool IsDigit(const std::wstring& strValue);
	/// @brief 是否字母
	static bool IsAlpha(const wchar_t& ch);
	static bool IsAlpha(const std::wstring& strValue);
	/// @brief 是否小写字母
	static bool IsLower(const wchar_t& ch);
	static bool IsLower(const std::wstring& strValue);
	/// @brief 是否大写字母
	static bool IsUpper(const wchar_t& ch);
	static bool IsUpper(const std::wstring& strValue);

	/// @brief 格式化字符串
	static std::string& StringFormat(std::string & _str, const char * _Format, ...);
	static std::wstring& WStringFormat(std::wstring & _str, const wchar_t * _Format, ...);

#ifdef QT
	static QString TCharToQString(const TCHAR* wp, size_t codePage = CP_ACP);
	static TCHAR* QStringToTChar(const QString &str);
	static std::string GBK2UTF8(const QString& qStr);
#endif

	static unsigned long getHashCode(const char* arkey, unsigned int nkeylength);
};
