/*
   字符指针数组演示
   通常采用指针数组记录多个相关字符串，因为每个指针之占4个字节
   */
#include <stdio.h>
int main() {
    //字符指针数组可以用来记录多个相关字符串
    char *strs[] = {"abc", "def", "hij", "klm", "xyz"};
    //二维数组也可以用来记录多个相关字符串
    //char strs[][4] = {"abc", "def", "hij", "klm", "xyz"};
    int num = 0;
    for (num = 0;num <= 4;num++){
        printf("%s\n", strs[num]);
    }
    return 0;
}
