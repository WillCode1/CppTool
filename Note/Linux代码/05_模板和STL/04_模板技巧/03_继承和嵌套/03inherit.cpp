//子类模板中访问基类模板的成员
#include <cstdlib>
#include <iostream>
#include <typeinfo>
using namespace std;
template<typename T>
class Base{
public:
    int m_var; // 成员变量
    void foo(void){ // 成员函数
        cout << m_var << endl;
    }
    class Type{}; // 成员类型
    void exit(int status){
        cout << "再见！" << endl;
    }
};
template<typename T>
class Derived:public Base<T>{
public:
    void bar(void){
//	Base<T>::m_var = 100;
        this->m_var = 100;
//	Base<T>::foo();
        this->foo();
        typename Base<T>::Type var;
        cout << typeid(var).name() << endl;
	//这里不加显示指明，编译器会误认为调用全局函数exit()
//	Base<T>::exit(0);
        this->exit(0);
    }
};
int main(void){
    Derived<int> d;
    d.bar();
    return 0;
}
