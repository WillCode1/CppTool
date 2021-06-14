//++Base64.h
#pragma once
#include <string>

class Base64
{
public:
	Base64() = default;
	~Base64() = default;

	/*编码
	DataByte
	[in]输入的数据长度,以字节为单位
	*/
	static std::string Encode(const char* Data, int DataByte);

	/*解码
	DataByte
	[in]输入的数据长度,以字节为单位
	OutByte
	[out]输出的数据长度,以字节为单位,请不要通过返回值计算
	输出数据的长度
	*/
	static std::string Decode(const char* Data, int DataByte, int& OutByte);
	static bool FileToBase64(const std::string& file_path, std::string& base64);
	static bool Base64ToFile(const std::string& file_path, const std::string& base64);
};

