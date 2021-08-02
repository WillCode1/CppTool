//零初始化
#include <iostream>
using namespace std;
template<typename T>

void foo(void){
    T var = T();
    cout << var << endl;
}
template<typename T>
class A{
public:
#if 1
    //类模板中的零初始化
    A (void):m_var(){
//  	m_var = T();
    }
#endif
    void foo(void)const{
        cout << m_var << endl;
    }
private:
    T m_var;
};

int main(void){
    foo<int>();
    foo<double>();
    foo<int*>();
    foo<string>();

    A<int> a1;
    a1.foo();
    A<double> a2;
    a2.foo();
    A<int*> a3;
    a3.foo();
    A<string> a4;
    a4.foo();
    return 0;
}
