/*
 * ~用法:二进制数每位数取反
 */
#include<stdio.h>

int main(){
    char a = 0x96;  //二进制数为1001 0110
    printf("%x\n",~a);//输出结果二进制为0110 1001
    return 0;
}
