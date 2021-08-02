//头文件卫士
#ifndef T_MATH_H 
#define T_MATH_H
//宏定义
//文件包含
//函数的声明
int add(int,int);
int sub(int,int);
int div(int,int);
int mul(int,int);

//变量的声明（注意区分变量的声明和定义）
//extern int var;
//类型的定义
struct node{
    int data;
    struct node *next;
};
typedef int count_t;
typedef int (*math_t)(int,int);

#endif
