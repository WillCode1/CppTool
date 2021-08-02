//静态成员变量
#include <iostream>
using namespace std;

class A{
public:
    int m_data;
//private:
    static int s_data;//声明
};
//必须在类的外部单独定义和初始化静态成员变量
int A::s_data = 100;//定义和初始化
int main(void)
{
    A a;
    cout << sizeof(a) << endl;//4
    //通过类名访问
    cout << A::s_data << endl;//100
    //通过对象访问
    cout << a.s_data << endl;//100
    return 0;
}

