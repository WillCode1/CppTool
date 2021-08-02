//对象类型函数返回值不是左值
//但是可以通过调用成员函数或者类型转换构造函数修改
#include <iostream>
using namespace std;

class A{
public:
    A(int data = 0){
        m_data = data;
    }
private:
    int m_data;
};

A foo(void){
    A a;
    return a;
}

int main(){
    A a(10);
    foo() = 100;//调用类型转换构造函数
    foo() = a;  //被编译器处理为拷贝赋值
    cout << &foo() << endl;//error
    return 0;
}
