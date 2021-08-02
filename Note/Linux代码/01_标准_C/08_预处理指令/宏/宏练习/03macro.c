/*
 * 宏练习，编写宏计算一个数的平方
 */
#include<stdio.h>
#define     SQUARE(n)     ((n) * (n))

int main() {
    int num = 0;
    float num1 = 0;
    printf("请输入一个整数，一个小数：");
    scanf("%d%f", &num, &num1);
    printf("SQUARE(num1)是%f\n", SQUARE(num1));
    printf("100 / SQUARE(num)是%d\n", 100 / SQUARE(num));
    printf("SQUARE(num + 3)是%d\n", SQUARE(num + 3));
    printf("SQUARE(++num)是%d\n", SQUARE(++num));
    return 0;
}
