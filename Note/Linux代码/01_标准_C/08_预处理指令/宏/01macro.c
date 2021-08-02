/*
    宏演示,宏给数字起名字
    程序中没有定义宏，编译时gcc -D
    例如:gcc -DPI=3.14f
   */
#include <stdio.h>
//#define   PI  3.14f
int main() {
    int radius = 0;
    printf("请输入半径：");
    scanf("%d", &radius);
    printf("周长是%g\n", 2 * PI * radius);
    return 0;
}
