//brk函数的使用
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int main(void)
{
    //获取一个有效位置
    void* p = sbrk(0);
    printf("p = %p\n",p);
    //使用brk函数申请4个字节内存
    int res = brk(p+4);
    if(-1 == res)
    {
        perror("brk"),exit(-1);
    }
    //获取当前位置
    void* cur = sbrk(0);
    printf("cur = %p\n",cur);//p+4
    //申请4个字节
    brk(p+8);
    cur = sbrk(0);
    printf("cur = %p\n",cur);//cur+4
    //释放了4个字节
    brk(p+4);
    cur = sbrk(0);
    printf("cur = %p\n",cur);//cur-4
    return 0;
}
