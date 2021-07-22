#pragma once
#ifndef _TIME_TOOL_H_
#define _TIME_TOOL_H_


long long getMseconds(void);
long long getUseconds(void);
void threadDelayS(const long long & seconds);
void threadDelayMs(const long long & milliseconds);
void threadDelayUs(const long long & microseconds);
void threadDelayNs(const long long & nanoseconds);
char *getTime();
bool getLocalTime(int *, int *);

#endif
