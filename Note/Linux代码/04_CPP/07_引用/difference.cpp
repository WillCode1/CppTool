//指针和引用的区别
#include <iostream>
using namespace std;

void func(int x, int y){return;}

int main(){
    //1.引用不允许先定义再赋值
    int x = 10, y = 20;
    int* p;
    p = &x;
    //int& r1;          //error
    
    //2.引用不能为空
    p = NULL;
    //int& r1 = NULL;   //error

    //3.引用不能更换目标
    int& r1 = x;
    r1 = y;             //表示把y赋值给x

    //4.可以定义二级指针,不能定义二级引用
    int** pp = &p;
    //int&& rr = r1;    //error
    
    //5.可以引用指针,不可以指向引用
    int*& r2 = p;
    //int&* p2 = &r;    //error
    
    //6.可以定义指针数组,不可以定义引用数组,却可以定义数组引用
    int* pa[5];
    //int& ra[5] = {...};   //error

    int a[5] = {1,2,3,4,5};
    int (&ar)[5] = a;       //数组引用
    int (*ap)[5] = &a;      //数组指针

    //7.函数指针和函数引用语法基本一样
    void (*pfunc)(int, int) = func; //函数指针
    void (&rfunc)(int, int) = func; //函数引用
    return 0;
}
