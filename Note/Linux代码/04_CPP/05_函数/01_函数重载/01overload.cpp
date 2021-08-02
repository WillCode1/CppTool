//函数重载
#include <iostream>
using namespace std;

double foo(void){
    cout << "foo(void)" << endl;
}
int foo(int a){
    cout << "foo(int)" << endl;
}
void foo(int a,int b){
    cout << "foo(int,int)" << endl;
}
void foo(int a,float b){
    cout << "foo(int,float)" << endl;
}
int main(void)
{
    foo(10);
    foo(10,20);
    foo();
    //foo(10,3.14); //error
    foo(10,3.14f);
    //函数指针的类型(返回值和参数表)决定重载版本
    void (*pfunc)(int,float) = foo;
    pfunc(10,20);
    return 0;
}
