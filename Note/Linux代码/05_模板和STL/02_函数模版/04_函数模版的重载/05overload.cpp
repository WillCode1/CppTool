//函数模版的重载
#include <cstring>
#include <iostream>
#include <typeinfo>
using namespace std;
// 两个任意类型值的最大值
template<typename T>
T const& max(T const& x, T const& y){
    cout << "<1" << typeid(x).name() << '>' << flush;
    return x < y ? y : x;
}
// 两个任意类型指针所指向目标的最大值
template<typename T>
T* const& max(T* const& x, T* const& y){
    cout << "<2" << typeid(x).name() << '>' << flush;
    return *x < *y ? y : x;
}
// 两个字符指针所指向字符串的最大值
char const* const& max(char const* const& x, char const* const& y){
    cout << "<3" << typeid(x).name() << '>' << flush;
    return strcmp (x, y) < 0 ? y : x;
}
// 三个任意类型值的最大值
template<typename T>
T const& max (T const& x, T const& y, T const& z){
    cout << "<4" << typeid(x).name() << '>' << flush;
    return ::max(::max(x, y), z);
}

int main (void) {
    int a = 123, b = 456;
    cout << ::max(a, b) << endl;
    cout << *::max(&a, &b) << endl;
    char const* x = "A";
    char const* y = "AB";
    cout << ::max(x, y) << endl;
    cout << ::max<>(x, y) << endl;  //<>选择模版函数
    cout << ::max<char const*>(x, y) << endl;
    char const* z = "ABC";
//  cout << ::max(x, y, z) << endl;
    char const* const& r = ::max(x, y, z);
    cout << r << endl;
    cout << "------------------------" << endl;
    char const* m = "1";
    char const* n = "12";
    char const* o = "123";
    ::max(m, n, o);
    cout << r << endl;
    return 0;
}
