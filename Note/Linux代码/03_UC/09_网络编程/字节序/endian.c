//测试大小端字节序
#include <stdio.h>

int main(){
    //int a = 305419896;
    //用十六进制表示,每两位表示一个字节
    int a = 0x12345678;
    char *p = (char *)&a;
    printf("%#x %#x %#x %#x\n",*p,*(p+1),*(p+2),*(p+3));
    return 0;
}
