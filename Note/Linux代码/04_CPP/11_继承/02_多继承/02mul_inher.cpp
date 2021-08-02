//多继承的名字冲突问题
#include <iostream>
using namespace std;
class A{
public:
    void foo(void){
        cout << "A::foo(void)" << endl;
    }
    int m_data;
};
class B{
public:
    void foo(int i){
        cout << "B::foo(int)" << endl;
    }
    typedef int m_data; //起一个别名
};
class C:public A,public B{
public:
    using A::foo;
    using B::foo;
};
int main(void)
{
    C c;
    c.A::foo();
    c.B::foo(100);
    //不同类型成员重名,也会报歧义错误
    c.A::m_data = 100;
    return 0;
}

