/*
   多文件编程演示
   编译时，gcc命令中提供所有源文件名称
   */
#include <stdio.h>
#include "01add.h"
int main() {
    int num = 0, num1 = 0, res = 0;
    printf("请输入两个数字：");
    scanf("%d%d", &num, &num1);
    res = add(num, num1);
    printf("求和结果是%d\n", res);
    return 0;
}
