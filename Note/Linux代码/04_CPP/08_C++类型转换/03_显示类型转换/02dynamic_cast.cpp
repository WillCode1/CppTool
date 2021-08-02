//动态类型转换,用于多态中父子类指针类型转换检查
#include <iostream>
using namespace std;
class A{virtual void foo(){}};
class B:public A{void foo(){}};
class C:public A{void foo(){}};
class D{};
int main(void)
{
    B b;
    A* pa = &b;
    B* pb = dynamic_cast<B*>(pa);
    cout << "pa=" << pa << endl;
    cout << "pb=" << pb << endl;
    //不合理向下造型，失败，返回NULL
    C* pc = dynamic_cast<C*>(pa);
    cout << "pc=" << pc << endl;
    D* pd = dynamic_cast<D*>(pa);
    cout << "pd=" << pd << endl;
    
    A& ra = b;
    //不合理向下造型，失败，抛出异常
    C& rc = dynamic_cast<C&>(ra);
    return 0;
}
