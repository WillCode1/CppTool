// 实现模板
#include <string>
using namespace std;
#include "cmp.h"
template<typename T>
T max (T x, T y) {
	return x < y ? y : x;
}
template<typename T>
T min (T x, T y) {
	return x < y ? x : y;
}
template<typename T>
Comparator<T>::Comparator (T x, T y) :
	m_x (x), m_y (y) {}
template<typename T>
T Comparator<T>::max (void) const {
	return m_x < m_y ? m_y : m_x;
}
template<typename T>
T Comparator<T>::min (void) const {
	return m_x < m_y ? m_x : m_y;
}
template int max<int> (int, int);
template int min<int> (int, int);
template double max<double> (double,
	double);
template double min<double> (double,
	double);
template string max<string> (string,
	string);
template string min<string> (string,
	string);
template class Comparator<int>;
template class Comparator<double>;
template class Comparator<string>;
