//类模板的成员虚函数
#include <iostream>
using namespace std;
template<typename A, typename B>
class X{
public:
    virtual A foo(B arg){
        cout << "X::foo" << endl;
    }
    template<typename C>
    //模板函数不能是virtual
    /*virtual */void bar(C arg){}
};
template<typename A, typename B, typename C>
//class Y:public X<A, B>{
class Y:public X<A, C>{
public:
    A foo(B arg){
        cout << "Y::foo" << endl;
    }
};
int main(void){
    Y<int, double, string> y;
//  X<int, double>& x = y;
    X<int, string>& x = y;
//  x.foo(1.23);
    x.foo("AB");
    x.bar<int>(10);
    return 0;
}
