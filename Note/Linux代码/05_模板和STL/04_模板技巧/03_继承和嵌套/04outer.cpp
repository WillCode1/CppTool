//内部模板的外部定义
#include <iostream>
#include <typeinfo>
using namespace std;
template<typename A>
class X{
public:
    // 类模板中的类模板
    /*
    template<typename B>
    class Y{
    public:
        // 类模板中的类模板中的函数模板
        template<typename C>
        void foo(void)const{
            C var;
        }
        B m_var;
    };
    */
    template<typename B> class Y;
    A m_var;
};
//外部定义的类模板
template<typename A>
    template<typename B>
class X<A>::Y{
public:
    template<typename C>
    void foo(void)const;
    B m_var;
};
//外部定义的函数模板
template<typename A>
    template<typename B>
        template<typename C>
void X<A>::Y<B>::foo(void)const{
    C var;
    cout << typeid(var).name() << endl;
}
int main(void){
    X<int> x;
    cout << typeid(x.m_var).name() << endl; // i
    X<int>::Y<double> y;
    cout << typeid(y.m_var).name() << endl; // d
    y.foo<string>(); // Ss
    return 0;
}
