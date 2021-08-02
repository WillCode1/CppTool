//隐式推断
#include <iostream>
#include <typeinfo>
using namespace std;
template<typename T>
void foo(T x, T y){
    cout << typeid(x).name() << ' ' << typeid(y).name() << endl;
}
template<typename T>
void bar(T const& x, T const& y){
    cout << typeid(x).name() << ' ' << typeid(y).name() << endl;
}
template<typename R, typename T>
R hum(T x){
    R y;
    cout << typeid(x).name() << ' ' << typeid(y).name() << endl;
    return y;
}
int main(void){
    int a, b;
    foo(a, b);  // i i
    double c, d;
    bar(c, d);  // d d
    char e[256], f[256];
    //表示数组首地址,隐式推断为字符指针
    foo(e, f);  // Pc Pc
    // e : char*
    
    //表示字符数组整体,隐式推断为数组指针
    foo (&e, &f);// PA256_c PA256_c
    // e : char[256]
    // &e : char (*)[256]
    
    bar (e, f); // A256_c A256_c
    // e : char[256]
    foo("hello", "tarena"); // PKc PKc
//  bar("hello", "tarena"); // A6_c A7_c
    // "hello" : const char[6]
    // "tarena" : const char[7]

    //隐式推断不能同时隐式转换
//  foo(a, c);
    foo<int>(a, c);     // c : double->int
    foo<double>(a, c);  // a : int->double
    foo(a, (int)c);     // i i
    foo((double)a, c);  // d d
    //返回值类型不能推断
    c = hum<double>(a); // i d
    return 0;
}
