/*
   联合演示
   */
#include <stdio.h>
typedef union {
    int num;
    float fnum;
} tmp;
int main() {
    tmp t1 = {0};
    printf("&(t1.num)是%p\n", &(t1.num));
    printf("&(t1.fnum)是%p\n", &(t1.fnum));
    printf("sizeof(tmp)是%d\n", sizeof(tmp));
    t1.num = 4;
    printf("t1.num是%d\n", t1.num);
    printf("t1.fnum是%f\n", t1.fnum);   //0.000000
    t1.fnum = 4.56;
    printf("t1.num是%d\n", t1.num);     //不确定
    printf("t1.fnum是%f\n", t1.fnum);
    return 0;
}
