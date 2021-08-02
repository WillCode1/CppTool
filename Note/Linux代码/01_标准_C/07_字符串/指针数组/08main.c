/*
   主函数参数演示,
   主函数的参数就使用字符指针数组记录了多个相关字符串
   */
#include <stdio.h>
int main(int argc, char *argv[]) {
    int num = 0;
    for (num = 0;num <= argc - 1;num++) {
        printf("%s\n", argv[num]);
    }
    return 0;
}
