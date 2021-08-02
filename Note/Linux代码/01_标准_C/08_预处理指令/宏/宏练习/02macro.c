/*
 * 宏练习，编写计算两个整数的差
 */
#include<stdio.h>
#define     SUB(x,y)    ((x) - (y))

int main(){
    printf("SUB(10, 5)是%d\n",SUB(10, 5));
    printf("20 - SUB(10, 5)是%d\n", 20 - SUB(10, 5));
    printf("SUB(20, 10 - 5)是%d\n", SUB(20, 10 - 5));
    return 0;
}
