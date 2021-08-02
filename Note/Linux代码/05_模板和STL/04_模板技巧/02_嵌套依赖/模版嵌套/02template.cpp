//template解决嵌套模板
#include <iostream>
#include <typeinfo>
using namespace std;
template<typename A>
class X{
public:
    // 类模板中的函数模板
    template<typename B>
    void foo(void)const{
        B var;
        cout << typeid(var).name() << ' ' 
            << typeid(m_var).name() << endl;
    }
    // 类模板中的类模板
    template<typename B>
    class Y{
    public:
        B m_var;
    };
    A m_var;
};

template<typename A, typename B>
void bar(void){
    X<A> x;
    x.template foo<B>();   // d i
    typename X<A>::template Y<B> y;
}
int main(void){
    X<int> x;
    x.foo<double>();        // d i

    X<int>::Y<double> y;

    bar<int, double>();
    return 0;
}
