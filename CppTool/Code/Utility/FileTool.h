#pragma once
#include <string>
#include <list>
#include <vector>

class FileTool
{
public:
	static std::string GetCurDirectory();	// 获取当前模块全路径
	static std::wstring GetDesktopDirectory();
	static bool ReadFileByLine(const std::string& filePath, std::vector<std::string>& all_line);
	static bool ReadFileByChar(const std::string& filePath, std::vector<char>& all_char, bool filter_space = true);

	static std::wstring GetFileNameNotPath(const std::wstring& filePath);//获取文件名，不包含路径
	static std::wstring GetFileExtension(const std::wstring & strPath);//获取文件的扩展名
	static std::wstring RemoveExtension(const std::wstring & strPath);//去除文件的扩展名
	static std::wstring GetFileNameNotExtension(const std::wstring & strPath);//获取文件名，不包含扩展名
	static std::wstring GetFileFolder(const std::wstring & strPath);//获取文件所在的文件夹路径
	static void copyFile(const char* src, const char* dst);//复制文件
	static int createDirectory(const wchar_t* dir);//创建目录，如果存在，则不会覆盖原来的文件
	static bool IsFileExist(const wchar_t* strPath);//判断文件，目录是否存在
	static bool IsFileExist(const char* strPath);//判断文件，目录是否存在
	static void GetFiles(std::string path, std::list<std::string>& files);//获取指定路径下的所有文件
	static bool DeleteFiles(std::string path);//删除指定目录下的所有文件，不删除该目录
	static bool DeleteFilesEx(std::string path);//删除指定目录下的所有文件，并且删除该目录
	static bool moveFile(std::string srcPath, std::string dstPath);//移动指定文件到指定路径

	static bool DeleteOldFiles(std::string path, int seconds);//遍历path文件夹，删除指定seconds之前修改的以fileExtension结尾的文件
	static std::wstring ToWindowSeprator(const std::wstring& s);	//转成Windows路径风格
};
