//typeid关键字,用于显示类型信息
#include <iostream>
#include <typeinfo>
using namespace std;
class X{virtual void foo(){}};
class Y:public X{void foo(){}};
class Z:public X{void foo(){}};

void func(X& x){
    if(typeid(x) == typeid(Y)){
        cout << "针对Y对象的处理" << endl;
    }
    else if(typeid(x) == typeid(Z)){
        cout << "针对Z对象的处理" << endl;
    }
}
int main(void)
{
    int x;
    cout << typeid(int).name() << endl; //i
    cout << typeid(x).name() << endl;   //i
    cout << typeid(char).name() << endl;//c
    cout << typeid(unsigned int).name() << endl;//j
    cout << typeid(double[10]).name() <<endl;   //A10_d
    cout << typeid(int* [5]).name() <<endl;     //A5_Pi
    cout << typeid(int(*)[5]).name() <<endl;    //PA5_i
    cout << typeid(int(*[5])(int)).name() << endl;//A5_PFiiE
    class AA{};
    cout << typeid(AA).name() << endl;//Z4mainE2AA
    X* px = new Y;
    cout << typeid(*px).name() << endl;//1Y

    Y y;
    func(y);
    Z z;
    func(z);
    return 0;
}
