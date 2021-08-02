/*
   函数指针演示
   */
#include <stdio.h>
int add(int num, int num1) {
    return num + num1;
}
int main() {
    int (*p_add)(int, int) = NULL;   //函数指针声明
    printf("add是%p\n", add);
    p_add = add;
    printf("p_add(3, 8)是%d\n", p_add(3, 8));
    return 0;
}
