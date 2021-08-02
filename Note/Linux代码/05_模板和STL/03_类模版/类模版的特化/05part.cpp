//类模版的局部特化
#include <iostream>
#include <typeinfo>
using namespace std;
// 通用版本
template<typename A, typename B>
class X{
public:
    static void foo(void){
        cout << "X<A,B>" << endl;
    }
};
// 完全特化
template<>
class X<int, short>{
public:
    static void foo(void){
        cout << "X<int,short>" << endl;
    }
};
// 以下都是局部特化
template<typename A>
//B = short
class X<A, short>{
public:
    static void foo(void){
        cout << "X<A,short>" << endl;
    }
};
template<typename A>
//B = A
class X<A, A>{
public:
    static void foo(void){
        cout << "X<A,A>" << endl;
    }
};
template<typename A>
//B = A*
class X<A, A*>{
public:
    static void foo(void){
        cout << "X<A,A*>" << endl;
    }
};
template<typename A, typename B>
//A = void*, B = void*
class X<A*, B*>{
public:
    static void foo(void){
        cout << "X<A*,B*>" << endl;
        cout << typeid(A).name() << ' ' 
            << typeid(B).name() <<endl;
    }
};
template<typename A>
//A = void*, B = A*
class X<A*, A*>{
public:
    static void foo(void){
        cout << "X<A*,A*>" << endl;
    }
};

int main (void) {
    X<int, double>::foo();
    X<int, short>::foo();
    X<double, short>::foo();
    X<int, int>::foo();
    X<double, double*>::foo();
    X<char***, int****>::foo();
    X<double*, double*>::foo();
    return 0;
}
