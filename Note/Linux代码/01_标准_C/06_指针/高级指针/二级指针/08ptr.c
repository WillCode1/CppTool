/*
   主函数的二级指针形参演示
   */
#include <stdio.h>
int main(int argc, char **argv) {
    int num = 0;
    for (num = 0;num <= argc - 1;num++) {
        printf("%s\n", *(argv + num));
    }
    return 0;
}
