//多态的条件
#include <iostream>
using namespace std;

class Base{
public:
    virtual int cal(int x,int y)const{
        return x + y;
    }
    //this指针是一个指向子类对象的基类指针
    void func(void){    //Base* this = &d
        cout << this->cal(100,200) << endl;
    }
};
class Derived:public Base{
public:
    int cal(int a,int b)const{
        return a * b;
    }
};
int main(void)
{
    Derived d;
    //Base b = d;
    //通过基类对象直接调用虚函数，没有多态
    //cout << b.cal(100,200) << endl;
    d.func();//this = &d
    return 0;
}

