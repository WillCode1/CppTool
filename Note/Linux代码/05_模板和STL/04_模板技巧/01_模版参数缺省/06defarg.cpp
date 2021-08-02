//模版参数的缺省值
#include <iostream>
#include <typeinfo>
using namespace std;
//类模版可以带有缺省值
template<typename A = int,typename B = double,typename C = string>
class X{
public:
    static void foo(void){
        cout << typeid(A).name() << ' ' << typeid(B).name() 
            << ' ' << typeid(C).name() <<endl;
    }
};
template<typename A = int,typename B = A>
class Y{
public:
    static void foo(void){
        cout << typeid(A).name() << ' '
            << typeid(B).name() <<endl;
    }
};
//函数模版可以带有缺省值
template<typename A = int,typename B = double,typename C = string>
void foo(void){
    cout << typeid(A).name() << ' ' << typeid(B).name() 
        << ' ' << typeid(C).name() <<endl;
}
template<typename T = int>
void bar(T arg){
    cout << typeid(arg).name() << endl;
}
int main(void){
    X<char, short, int>::foo(); // c s i
    X<char, short>::foo();      // c s Ss
    X<char>::foo();             // c d Ss
    X<>::foo();                 // i d Ss
//  X::foo();   //error
    Y<int>::foo();              // i i

    foo<char,short,int>();  // c s i
    foo<char,short>();      // c s Ss
    foo<char>();            // c d Ss
    foo<>();                // i d Ss
    foo();                  // i d Ss
    bar(3.14);              // d
    return 0;
}
