//缺省参数
#include <iostream>
using namespace std;
//函数声明，缺省值只能写在函数声明
void foo(int a,int b=200,int c=300);
//注意防止重载的歧义冲突
//void foo(int i){}
//不能用局部变量作为函数参数的缺省值
//void foo(int a, int b = a){}

int main()
{
    foo(1);//1,200,300
    foo(1,2);//1,2,300
    foo(1,2,3);//1,2,3
    return 0;
}
//函数定义,标明函数有缺省值,方便使用
void foo(int a,int b/*=200*/,int c/*=300*/){
    cout << "a=" << a << ',' << "b=" << b << ',' << "c=" << c << endl;
}
