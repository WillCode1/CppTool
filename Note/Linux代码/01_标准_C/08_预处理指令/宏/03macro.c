/*
   宏演示
   */
#include <stdio.h>
#define ABS(n)  n >= 0 ? n : 0 - n
#define NEG(n)  n = 0 - n
int abs(int num) {
    if (num >= 0) {
        return num;
    }
    else {
        return 0 - num;
    }
}
void neg(int *p_num) {
    *p_num = 0 - *p_num;
}
int main() {
    int num = 0;
    printf("请输入一个整数：");
    scanf("%d", &num);
    printf("绝对值是%d\n", abs(num));
    printf("绝对值是%d\n", ABS(num));
    neg(&num);
    printf("num是%d\n", num);
    NEG(num);
    printf("num是%d\n", num);
    return 0;
}
