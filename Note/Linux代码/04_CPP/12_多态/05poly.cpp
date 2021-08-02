//函数签名相同,返回类型不同
#include <iostream>
using namespace std;
class X{
public:
    virtual void foo(int arg){
        cout << "X::foo" << endl;
    }
};
class Y:public X{
public:
    int foo(int arg){
    //void foo(int arg){
        cout << "Y::foo" << endl;
    }
};

int main(void){
    Y y;
    X& x = y;
    x.foo(7);
    return 0;
}
