/*
   数据对齐和补齐演示
   */
#include <stdio.h>
typedef struct {    //2(+2)+4=8,数据对齐
    char buf[2];
    int num;
} tmp;
typedef struct {    //1(+3)+4+1(+3)=12,数据补齐
    char ch;
    int num;
    char ch1;
} tmp1;
typedef struct {    //1+1+2+4=8
    char ch;
    char ch1;
    char buf[2];
    int num;
} tmp2;
int main() {
    printf("sizeof(tmp)是%d\n", sizeof(tmp));
    printf("sizeof(tmp1)是%d\n", sizeof(tmp1));
    printf("sizeof(tmp2)是%d\n", sizeof(tmp2));
    return 0;
}
