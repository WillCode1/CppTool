//拷贝构造执行顺序
#include <iostream>
using namespace std;
//基类
class A{
public:
    A(int data=0):m_num1(data){
        cout << "A::构造函数" << endl;
    }
    A(const A& that):m_num1(that.m_num1){
        cout << "A::拷贝构造" << endl;
    }
    int m_num1;
};
class C{
public:
    C(int data=0):m_num2(data){
        cout << "C::构造函数" << endl;
    }
    C(const C& that):m_num2(that.m_num2){
        cout << "C::拷贝构造" << endl;
    }
    int m_num2;
};
//子类
class B:public A{
public:
    B(int x=0,int y=0,int z=0):A(x),m_c(y),m_num3(z){
        cout << "B::构造函数" << endl;
    }
    B(B const& that):
    //初始化表中,分别调用基类子对象和成员子对象的拷贝构造函数
        A(that),m_c(that.m_c),m_num3(that.m_num3){
        cout << "B::拷贝构造" << endl;
    }
    C m_c;  //成员子对象
    int m_num3;;
};

int main(void)
{
    B b(1,2,3);
    cout << "--------------------" << endl;
    B b2(b);//调用拷贝构造函数
    cout << b2.m_num1 << b2.m_c.m_num2 << b2.m_num3 << endl;
    return 0;
}

