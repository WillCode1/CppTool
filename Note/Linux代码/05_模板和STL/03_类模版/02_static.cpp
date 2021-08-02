//类模版的静态成员
#include <iostream>
using namespace std;
//只要有类模版参数，不使用也是类模版
template<typename T>
class A{
public:
    void foo(void)const{
        cout << "this: " << this << ", &m_var: " << &m_var
            << ", &s_var: " << &s_var << endl;
    }
private:
    int m_var;
    static int s_var;
};
template<typename T>
int A<T>::s_var;

int main(void){
    static A<int> a, b;
    static A<double> c, d;
    a.foo();
    b.foo();
    c.foo();
    d.foo();
}
