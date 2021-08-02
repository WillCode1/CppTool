//错误处理,远程跳转机制
#include <iostream>
#include <cstdio>
#include <csetjmp>
using namespace std;

jmp_buf g_env;

class A{
public:
    A(void){cout << "A::A()" << endl;}
    ~A(void){cout << "A::~A()" << endl;}
};
int func3(void){
    A a;
    FILE* fp = fopen("none.txt","r");
    if(fp == NULL){
        longjmp(g_env,-1);
    }
    //...
    fclose(fp);
    return 0;
}
int func2(void){
    A a;
    func3();
    //...
    return 0;
}
int func1(void){
    A a;
    func2();
    //...
    return 0;
}
int main(void)
{
    if(setjmp(g_env)==-1){
        cout << "file open failed" << endl;
        return -1;
    }
    func1();
    return 0;
}

