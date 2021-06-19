#pragma once
#include <ctime>


class SystemDetection
{
public:
	/*********************************************************
	 * 使用类型的强制转换实现little-endian与big-endian的判断
	 *********************************************************
	 * 返回值：
	 *          1 表示是小端字节序。
	 *          0 表示不是小端字节序。
	 *********************************************************/
	static bool IsLittleEndian()
	{
		unsigned short flag = 0x4321;
		if (*(unsigned char*)&flag == 0x21)
			return true;
		else
			return false;
	}
};
