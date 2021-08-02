// 实例化模板
#include <iostream>
using namespace std;
#include "cmp.h"
int main (void) {
	int a = 123, b = 456;
	double c = 1.3, d = 4.6;
	string e = "hello", f = "world";
	cout << ::max (a, b) << ' '
		<< ::min (a, b) << endl;
	cout << ::max (c, d) << ' '
		<< ::min (c, d) << endl;
	cout << ::max (e, f) << ' '
		<< ::min (e, f) << endl;
	Comparator<int> ci (a, b);
	cout << ci.max () << ' '
		<< ci.min () << endl;
	Comparator<double> cd (c, d);
	cout << cd.max () << ' '
		<< cd.min () << endl;
	Comparator<string> cs (e, f);
	cout << cs.max () << ' '
		<< cs.min () << endl;
	return 0;
}
