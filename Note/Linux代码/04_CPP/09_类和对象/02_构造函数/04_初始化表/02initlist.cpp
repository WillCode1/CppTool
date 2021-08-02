//只能使用初始化表的情况
//情况1：成员子对象没有无参构造函数
#include <iostream>
using namespace std;

class A{
public:
    //A(void){} //无参构造
    A(int data){
        cout << "A::构造函数" << endl;
        m_data = data;
    }
    int m_data;
};
class B{
public:
    //因为成员子对象的构造函数先调用
    //通过初始化表给成员子对象的有参构造函数传参
    B(void):m_a(100){
        //m_a = 100;
        cout << "B::构造函数" << endl;
    }
    A m_a;
};

int main(void)
{
    B b;
    cout << b.m_a.m_data << endl;//100
    return 0;
}


