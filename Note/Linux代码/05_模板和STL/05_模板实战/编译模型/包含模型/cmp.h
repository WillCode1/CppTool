// 声明模板
#ifndef _CMP_H
#define _CMP_H
template<typename T>
T max (T x, T y);
template<typename T>
T min (T x, T y);
template<typename T>
class Comparator {
public:
	Comparator (T x, T y);
	T max (void) const;
	T min (void) const;
private:
	T m_x, m_y;
};
#include "cmp.cpp"
#endif // _CMP_H
