//覆盖的条件
#include <iostream>
using namespace std;
class A{};
class B:public A{};
class Base{
public:
    virtual void foo(void){
        cout << "Base::foo" << endl;
    }
    virtual void bar(void){
        cout << "Base::bar" << endl;
    }
    virtual A* hum(int a = 0){
        cout << "Base::hum" << endl;
    }
};
class Derived:public Base{
public:
    //返回类型是类类型指针或引用，并且具有继承关系
    //可以形成协变覆盖
    B* hum(int a = 0){
        cout << "Derived::hum" << endl;
    }
    void bar(void)const{
        cout << "Derived::bar(const)" << endl;
    }
    void bar(int x){
        cout << "Derived::bar" << endl;
    }
private:
    /*virtual*/ void foo(void){
        cout << "Derived::foo" << endl;
    }
};
int main(void)
{
    Base* pb = new Derived;
    pb->foo();//骗过了编译器,可以访问子类私有函数
    pb->bar();
    pb->hum();
    return 0;
}

