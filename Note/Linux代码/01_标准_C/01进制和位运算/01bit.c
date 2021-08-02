/*
 * 八进制数前加0，十六进制数前加0x，让计算机识别
 */
#include<stdio.h>
int main(){
    printf("%#o %o\n",0152,152);
    printf("%#x %X\n",0xcb ,0xcb);
    return 0;
}
