#pragma once
#include <ctime>
#include <iostream>
#include <string>
#include <fstream>
#include <shlobj.h>
#include <assert.h>
using namespace std;


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

	std::string GetFileName(const std::string& filePath)
	{
		if (filePath.rfind("\\") != filePath.npos)
		{
			return filePath.substr(filePath.rfind("\\") + 1).c_str();
		}
		return filePath;
	}

	std::wstring GetDesktopDirectory()
	{
		TCHAR path[128];
		//获取当前用户的桌面路径
		SHGetSpecialFolderPath(0, path, CSIDL_DESKTOPDIRECTORY, 0);
		return path;
	}
};

class RunTimeTool
{
public:
	void StartTimer()
	{
		startTime = clock();
		lastTime = clock();
	}

	double TotalCostTime() const
	{
		double total_time = static_cast<double>(clock() - startTime) / CLOCKS_PER_SEC;
		std::cout << "总计耗时：" << total_time << std::endl;
		return total_time;
	}

	double ElapsedTime()
	{
		double total_time = static_cast<double>(clock() - lastTime) / CLOCKS_PER_SEC;
		std::cout << "间隔耗时：" << total_time << std::endl;
		lastTime = clock();
		return total_time;
	}

private:
	clock_t startTime = 0;
	clock_t lastTime = 0;
};

class DebugLogTool
{
private:
	DebugLogTool(const wstring& path = GetDesktopDirectory())
	{
		logPath = path + L"\\DebugLog.txt";
		ofstream outfile(logPath, ios::trunc);
		outfile.close();
	}
	DebugLogTool(const DebugLogTool& other) = delete;
	DebugLogTool& operator=(DebugLogTool&) = delete;
	DebugLogTool& operator=(DebugLogTool&&) = delete;

public:
	static DebugLogTool& GetInstance(const wstring& path = GetDesktopDirectory())
	{
		static DebugLogTool tool(path);
		return tool;
	}

	template<typename T>
	void DebugLog(const T data, int newLineNum = 0)
	{
		ofstream outfile(logPath, ios::app);

		if (!outfile.is_open())
		{
			cout << "can not open log file" << endl;
			return;
		}

		outfile << data;
		cout << data;

		while (newLineNum--)
		{
			outfile << endl;
			cout << endl;
		}

		outfile.close();
	}

	template <>
	void DebugLog(const wstring data, int newLineNum)
	{
		DebugLog(WStrToStr(data), newLineNum);
	}

	template <>
	void DebugLog(const wchar_t* data, int newLineNum)
	{
		DebugLog<wstring>(data, newLineNum);
	}

private:
	wstring logPath;
};

#define DEBUG_LOG(data)     \
	do {						\
		DebugLogTool& tool = DebugLogTool::GetInstance();		\
		tool.DebugLog("FileName: ");	tool.DebugLog(GetFileName(__FILE__));	\
		tool.DebugLog(", Function: ");	tool.DebugLog(__FUNCTION__);	\
		tool.DebugLog(", Line: ");		tool.DebugLog(__LINE__);	\
		tool.DebugLog(", Content: ");	tool.DebugLog(data, 2);	\
	} while(0)

char *getTime();

#define LU_DEBUG_CMD 0x01
#define LU_DEBUG_DATA 0x02
#define LU_DEBUG_ERROR 0x04

int g_lu_debugs_level = LU_DEBUG_ERROR;
#define LU_PRINTF_cmd(...)             \
  do                                      \
  {                                       \
    if (g_lu_debugs_level & LU_DEBUG_CMD) \
    {                                     \
      printf("%s", getTime());            \
      printf(__VA_ARGS__);                        \
    }                                     \
  } while (0)
#define LU_PRINTF_data(...)             \
  do                                       \
  {                                        \
    if (g_lu_debugs_level & LU_DEBUG_DATA) \
    {                                      \
      printf("%s", getTime());             \
      printf(__VA_ARGS__);                         \
    }                                      \
  } while (0)
#define LU_PRINTF_error(...)             \
  do                                        \
  {                                         \
    if (g_lu_debugs_level & LU_DEBUG_ERROR) \
    {                                       \
      printf("%s", getTime());              \
      printf(__VA_ARGS__);                          \
    }                                       \
  } while (0)
//#define LU_PRINTF_cmd(msg...)  ;
//#define LU_PRINTF_data(msg...)  ;
//#define LU_PRINTF_error(msg...)  ;


#define lu_printf(level, ...) LU_PRINTF_##level(__VA_ARGS__)
