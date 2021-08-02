//隐式类型转换,发生的情况
#include <iostream>
using namespace std;

int foo(int n){     //2.传参
    char c = 'A';
    return c;       //3.函数返回值
}

void func(void* p){
    cout << "func" << endl;
}

int main(){
    char c = 'a';
    int a = 10;
    double d = a + c;   //1.赋值
    cout << d << endl;
    a = foo(c);         //char --> int,小转大
    cout << a << endl;
    a = foo(d);         //double --> int,大转小
    cout << a << endl;

    int* p = &a;
    func(p);    //int* --> void*,反之不可行
    return 0;
}
