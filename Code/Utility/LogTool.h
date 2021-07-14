#pragma once
#include "TimeTool.h"

extern int g_lu_debugs_level;

enum LogLevel
{
	cmdDebug = 0x01,
	dataDebug = 0x02,
	errorDebug = 0x04
};

#define LOG_PRINT(level, ...)	\
  do                                      \
  {                                       \
    if (g_lu_debugs_level & level)	\
    {                                     \
      printf("%s", getTime());            \
      printf(__VA_ARGS__);                 \
    }                                     \
  } while (0)

#define log_print(level, ...) LOG_PRINT(level##Debug, __VA_ARGS__)
