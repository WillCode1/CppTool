//静态成员函数
#include <iostream>
using namespace std;

class A{
public:
    static void func(void){
        cout << "静态成员函数" << endl;
        cout << s_data << endl;
        //cout << m_data << endl;
    }
    void func2(void){
        cout << "非静态成员函数" << endl;
        cout << s_data << endl;
        cout << m_data << endl;
    }
    int m_data;
    static int s_data;
};
int A::s_data = 100;

int main(void)
{
    //A::func();
    A a;
    a.m_data = 200;
    a.func();
    a.func2();
    return 0;
}

