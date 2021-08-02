//typename解决类型嵌套
//告诉编译器,后面跟着的是数据类型
#include <iostream>
using namespace std;
template<typename T>
class A {   // 包装类/外部类
public:
    class B { // 嵌套类/内部类
    public:
        T m_var;
    };
    T m_var;
};
template<typename T>
void foo (T arg) {
    A<T> a;
    cout << sizeof (a) << endl; // 4
    typename A<T>::B b;         // 嵌套依赖
    cout << sizeof (b) << endl; // 4
}
template<typename T>
class Z {
public:
    typename A<T>::B m_var;     // 嵌套依赖
};
int main (void) {
    A<int> a;
    cout << sizeof (a) << endl; // 4
    A<int>::B b;
    cout << sizeof (b) << endl; // 4
    foo (100);
    Z<int> z;
    cout << sizeof (z.m_var) << endl;// 4
};
