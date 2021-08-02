//大小端字节序的判断
#include <stdio.h>

int main(void)
{
    union Un
    {
        int x;
        char buf[4];
    };
    union Un un;
    un.x = 0x12345678;
    int i = 0;
    for(i = 0; i < 4; i++)
    {
        printf("%#x[%p] ",un.buf[i],&un.buf[i]);
    }
    printf("\n");
    return 0;
}
