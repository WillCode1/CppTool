//只能使用初始化表的情况
//情况2：包含常量或者引用类型的变量
#include <iostream>
using namespace std;

int g_data = 100;

class A{
public:
	A(void):m_r(g_data),m_c(200){}
	int& m_r;
	const int m_c;
};
int main(void)
{
	A a;
	cout << a.m_r << ' ' << a.m_c << endl;
	return 0;
}








