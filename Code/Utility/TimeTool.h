#pragma once
#ifndef _TIME_TOOL_H_
#define _TIME_TOOL_H_


long long getMseconds(void);
long long getUseconds(void);
bool threadDelayUs(const long usec);
bool threadDelayMs(const long msec);
bool threadDelayS(const long sec);
char *getTime();
bool getLocalTime(int *, int *);

#endif
