#pragma once
#include <cstdio>
#include "TimeTool.hpp"


// https://blog.csdn.net/q343509740/article/details/79726708
/*
	All:最低等级的，用于打开所有日志记录.

	Trace:是追踪，就是程序推进一下.

	Debug:指出细粒度信息事件对调试应用程序是非常有帮助的.

	Info:消息在粗粒度级别上突出强调应用程序的运行过程.

	Warn:输出警告及warn以下级别的日志.

	Error:输出错误信息日志.

	Fatal:输出每个严重的错误事件将会导致应用程序的退出的日志.

	OFF:最高等级的，用于关闭所有日志记录.
*/
enum LogLevel
{
	debug_log = 0x01,
	info_log = 0x02,
	error_log = 0x04
};


//#define	LOG_PRINT_LEVEL	(0)
//#define	LOG_PRINT_LEVEL	(error_log)
#define	LOG_PRINT_LEVEL	(debug_log | error_log)

#define log_print(level, ...)	\
  do										\
  {											\
    if (LOG_PRINT_LEVEL & level##_log)	\
    {										\
      printf("%s", TimeTool::GetLocalTimeStamp());		\
      printf("%s: Line %d:\t", __FUNCTION__, __LINE__);	\
      printf(__VA_ARGS__);                 \
      printf("\n");						\
    }                                     \
  } while (0)

// 打印宽字节字符串
//printf("%s\n", a.c_str());
//printf("%S\n", b.c_str());

//wprintf(L"%s\n", b.c_str());
//wprintf(L"%S\n", a.c_str());

