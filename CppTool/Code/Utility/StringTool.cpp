// 字符串转换工具
#include "StringTool.h"
#include <Windows.h>
#include <algorithm>


std::string StringTool::Wchar_tToString(const wchar_t* wchar)
{
	const wchar_t* wText = wchar;
	const DWORD dwNum = WideCharToMultiByte(CP_OEMCP, NULL, wText, -1, nullptr, 0, nullptr, FALSE);
	const auto psText = new char[dwNum];
	WideCharToMultiByte(CP_OEMCP, NULL, wText, -1, psText, dwNum, nullptr, FALSE);
	std::string res = psText;
	delete[] psText;
	return res;
}

std::string StringTool::UTF8ToGBK(const std::string& strUtf8)
{
	int len = MultiByteToWideChar(CP_UTF8, 0, strUtf8.c_str(), -1, NULL, 0);
	wchar_t* wszGBK = new wchar_t[len];
	memset(wszGBK, 0, len);
	MultiByteToWideChar(CP_UTF8, 0, strUtf8.c_str(), -1, wszGBK, len);

	len = WideCharToMultiByte(CP_ACP, 0, wszGBK, -1, NULL, 0, NULL, NULL);
	char* szGBK = new char[len + 1];
	memset(szGBK, 0, len + 1);
	WideCharToMultiByte(CP_ACP, 0, wszGBK, -1, szGBK, len, NULL, NULL);

	std::string res = szGBK;
	delete[] szGBK;
	delete[] wszGBK;
	return res;
}

std::string StringTool::GBKToUTF8(const std::string& strGBK)
{
	int len = MultiByteToWideChar(CP_ACP, 0, strGBK.c_str(), -1, NULL, 0);
	wchar_t* wszUtf8 = new wchar_t[len];
	memset(wszUtf8, 0, len);
	MultiByteToWideChar(CP_ACP, 0, strGBK.c_str(), -1, wszUtf8, len);

	len = WideCharToMultiByte(CP_UTF8, 0, wszUtf8, -1, NULL, 0, NULL, NULL);
	char* szUtf8 = new char[len + 1];
	memset(szUtf8, 0, len + 1);
	WideCharToMultiByte(CP_UTF8, 0, wszUtf8, -1, szUtf8, len, NULL, NULL);

	std::string res = szUtf8;
	delete[] szUtf8;
	delete[] wszUtf8;
	return res;
}

std::wstring StringTool::StrToWStr(const std::string& strValue)
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

std::string StringTool::WStrToStr(const std::wstring& strValue)
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

std::wstring StringTool::UTF8ToUnicode(const std::string& str)
{
	int  len = 0;
	len = str.length();
	int  unicodeLen = ::MultiByteToWideChar(CP_UTF8,
		0,
		str.c_str(),
		-1,
		NULL,
		0);
	wchar_t* pUnicode;
	pUnicode = new  wchar_t[unicodeLen + 1];
	memset(pUnicode, 0, (unicodeLen + 1) * sizeof(wchar_t));
	::MultiByteToWideChar(CP_UTF8,
		0,
		str.c_str(),
		-1,
		(LPWSTR)pUnicode,
		unicodeLen);
	std::wstring  rt;
	rt = (wchar_t*)pUnicode;
	delete  pUnicode;

	return  rt;
}

std::string StringTool::UnicodeToUTF8(const std::wstring& str)
{
	char* pElementText;
	int    iTextLen;
	// wide char to multi char
	iTextLen = WideCharToMultiByte(CP_UTF8,
		0,
		str.c_str(),
		-1,
		NULL,
		0,
		NULL,
		NULL);
	pElementText = new char[iTextLen + 1];
	memset((void*)pElementText, 0, sizeof(char) * (iTextLen + 1));
	::WideCharToMultiByte(CP_UTF8,
		0,
		str.c_str(),
		-1,
		pElementText,
		iTextLen,
		NULL,
		NULL);
	std::string strText;
	strText = pElementText;
	delete[] pElementText;
	return strText;
}

std::wstring StringTool::ANSIToUnicode(const std::string& str)
{
	int len = 0;
	len = str.length();
	int unicodeLen = ::MultiByteToWideChar(CP_ACP,
		0,
		str.c_str(),
		-1,
		NULL,
		0);
	wchar_t* pUnicode;
	pUnicode = new wchar_t[unicodeLen + 1];
	memset(pUnicode, 0, (unicodeLen + 1) * sizeof(wchar_t));
	::MultiByteToWideChar(CP_ACP,
		0,
		str.c_str(),
		-1,
		(LPWSTR)pUnicode,
		unicodeLen);
	std::wstring rt;
	rt = (wchar_t*)pUnicode;
	delete pUnicode;
	return rt;
}

std::string StringTool::UnicodeToANSI(const std::wstring& str)
{
	char* pElementText;
	int    iTextLen;
	// wide char to multi char
	iTextLen = WideCharToMultiByte(CP_ACP,
		0,
		str.c_str(),
		-1,
		NULL,
		0,
		NULL,
		NULL);
	pElementText = new char[iTextLen + 1];
	memset((void*)pElementText, 0, sizeof(char) * (iTextLen + 1));
	::WideCharToMultiByte(CP_ACP,
		0,
		str.c_str(),
		-1,
		pElementText,
		iTextLen,
		NULL,
		NULL);
	std::string strText;
	strText = pElementText;
	delete[] pElementText;
	return strText;
}

int StringTool::StrToInt(const std::wstring& strValue)
{
	return _wtoi(strValue.c_str());
}

double StringTool::StrToDouble(const std::wstring& strValue)
{
	return _wtof(strValue.c_str());
}

std::wstring StringTool::IntToStr(int nValue, int nRadix)
{
	if (nRadix != 2 && nRadix != 8 && nRadix != 10 && nRadix != 16)
	{
		nRadix == 10;
	}

	wchar_t wsValue[20] = { 0 };
	//swprintf_s(wsValue, L"%d", nValue);
	_itow_s(nValue, wsValue, nRadix);

	return wsValue;
}

std::wstring StringTool::DoubleToStr(const double& dValue, int accuracy)
{
	accuracy = accuracy < 0 ? 8 : accuracy;

	std::wstring strFormat = L"%." + IntToStr(accuracy) + L"f";
	wchar_t wsValue[20] = { 0 };
	swprintf_s(wsValue, strFormat.c_str(), dValue);

	return wsValue;
}

bool StringTool::SplitStr(const std::wstring& str, const std::wstring& tag,
	std::vector<std::wstring>& vec)
{
	if (L"" == str)
	{
		return false;
	}
	//方便截取最后一段数据
	std::wstring strs = str + tag;

	int pos = strs.find(tag);
	int size = strs.size();

	while (pos != std::wstring::npos)
	{
		std::wstring x = strs.substr(0, pos);
		vec.push_back(x);
		strs = strs.substr(pos + 1, size);
		pos = strs.find(tag);
	}

	return (vec.size() > 0);
}

std::wstring StringTool::ToLower(const std::wstring& strValue)
{
	std::wstring temp = strValue;
	std::transform(temp.begin(), temp.end(), temp.begin(), tolower);
	return temp;
}

std::wstring StringTool::ToUpper(const std::wstring& strValue)
{
	std::wstring temp = strValue;
	std::transform(temp.begin(), temp.end(), temp.begin(), toupper);
	return temp;
}

bool StringTool::SpanIncluding(const std::wstring& strValue, const std::wstring& strInclude)
{
	int nCount = strValue.length();
	for (int nIndex = 0; nIndex < nCount; ++nIndex)
	{
		if (strInclude.find(strValue.at(nIndex)) == std::wstring::npos)
		{
			return false;
		}
	}

	return true;
}

bool StringTool::IsDigit(const wchar_t& ch)
{
	return isdigit(ch);
}

bool StringTool::IsDigit(const std::wstring& strValue)
{
	std::wstring::const_iterator itCh = strValue.begin();
	for (; itCh != strValue.end(); ++itCh)
	{
		if (!IsDigit(*itCh))
		{
			return false;
		}
	}

	return true;
}

bool StringTool::IsAlpha(const wchar_t& ch)
{
	return isalpha(ch);
}

bool StringTool::IsAlpha(const std::wstring& strValue)
{
	std::wstring::const_iterator itCh = strValue.begin();
	for (; itCh != strValue.end(); ++itCh)
	{
		if (!IsAlpha(*itCh))
		{
			return false;
		}
	}

	return true;
}

bool StringTool::IsLower(const wchar_t& ch)
{
	return islower(ch);
}

bool StringTool::IsLower(const std::wstring& strValue)
{
	std::wstring::const_iterator itCh = strValue.begin();
	for (; itCh != strValue.end(); ++itCh)
	{
		if (!IsLower(*itCh))
		{
			return false;
		}
	}

	return true;
}

bool StringTool::IsUpper(const wchar_t& ch)
{
	return isupper(ch);
}

bool StringTool::IsUpper(const std::wstring& strValue)
{
	std::wstring::const_iterator itCh = strValue.begin();
	for (; itCh != strValue.end(); ++itCh)
	{
		if (!IsUpper(*itCh))
		{
			return false;
		}
	}

	return true;
}

std::string& StringTool::StringFormat(std::string & _str, const char * _Format, ...)
{
	std::string tmp;
	va_list marker = NULL;
	va_start(marker, _Format);
	size_t num_of_chars = _vscprintf(_Format, marker);
	if (num_of_chars > tmp.capacity()) {
		tmp.resize(num_of_chars + 1);
	}
	vsprintf_s((char *)tmp.data(), tmp.capacity(), _Format, marker);
	va_end(marker);
	_str = tmp.c_str();
	return _str;
}

std::wstring& StringTool::WStringFormat(std::wstring & _str, const wchar_t * _Format, ...)
{
	std::wstring tmp;
	va_list marker = NULL;
	va_start(marker, _Format);
	size_t num_of_chars = _vscwprintf(_Format, marker);
	if (num_of_chars > tmp.capacity()) {
		tmp.resize(num_of_chars + 1);
	}
	vswprintf_s((wchar_t *)tmp.data(), tmp.capacity(), _Format, marker);
	va_end(marker);
	_str = tmp.c_str();
	return _str;
}

unsigned long StringTool::getHashCode(const char* arkey, unsigned int nkeylength)
{
	unsigned long h = 0, g;
	const char* arEnd = arkey + nkeylength;

	while (arkey < arEnd)
	{
		h = (h << 4) + *arkey++;
		if ((g = (h & 0xF0000000)))
		{
			h = h ^ (g >> 24);
			h = h ^ g;
		}
	}
	return h;
}

#ifdef QT
QString StringConvertTools::TCharToQString(const TCHAR* wp, size_t codePage = CP_ACP)
{
	QString str;
	int len = WideCharToMultiByte(codePage, 0, wp, wcslen(wp), NULL, 0, NULL, NULL);
	char *p = new char[len + 1];
	memset(p, 0, len + 1);
	WideCharToMultiByte(codePage, 0, wp, wcslen(wp), p, len, NULL, NULL);
	p[len] = '\0';
	str = QString(p);
	delete p;
	p = NULL;
	return str;
}

TCHAR* StringConvertTools::QStringToTChar(const QString &str)
{
	QByteArray ba = str.toUtf8();
	char *data = ba.data(); //以上两步不能直接简化为“char *data = str.toUtf8().data();”
	int charLen = strlen(data);
	int len = MultiByteToWideChar(CP_ACP, 0, data, charLen, NULL, 0);
	TCHAR *buf = new TCHAR[len + 1];
	MultiByteToWideChar(CP_ACP, 0, data, charLen, buf, len);
	buf[len] = '\0';
	return buf;
}

std::string StringConvertTools::GBK2UTF8(const QString& qStr)
{
	return qStr.toUtf8().data();
}
#endif
