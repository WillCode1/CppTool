/*
   字符串函数演示
   atoi,atof
*/
#include <stdio.h>
#include <stdlib.h>
int main() {
    double dnum = 0.0;
    int num = atoi("34dsaklgj");    //记录字符串中不带小数点的数字
    printf("num是%d\n", num);
//    num = atoi("23.34dasd");
//    printf("num是%d\n", num);
    dnum = atof("56.23sdgfds");     //记录字符串中带小数点的数字
    printf("dnum是%lg\n", dnum);
    return 0;
}
