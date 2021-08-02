//对象从创建到销毁的全过程
#include <iostream>
using namespace std;
//基类
class A{
public:
    A(int x=0):m_num1(x){cout << "A::构造函数" << endl;}
    ~A(void){cout << "A::析构函数" << endl;}
    int m_num1;
};
class B{
public:
    B(int y=0):m_num2(y){cout << "B::构造函数" << endl;}
    ~B(void){cout << "B::析构函数" << endl;}
    int m_num2;
};
//子类
class C:public A{
public:
    C(int x=0,int y=0):A(x),m_b(y){cout << "C::构造函数" << endl;}
    ~C(void){cout << "C::析构函数" << endl;}
    B m_b;  //成员子对象
};

int main(void)
{
#if 1
    C c(100,200);
    //delete一个指向子类对象的基类指针,会导致内存泄露
#else
    A* pa = new C;
    delete pa;
#endif
    return 0;
}

