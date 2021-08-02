//映射的建立和删除
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>

int main(void)
{
    //1.使用mmap建立到物理内存的映射
    void* p = mmap(NULL/*由内核指定*/,4/*映射的大小*/,
    PROT_READ|PROT_WRITE/*权限*/,MAP_PRIVATE|MAP_ANONYMOUS/*模式*/,
    0/*文件描述符*/, 0/*文件的偏移量*/);
    if(p == MAP_FAILED)
    {
        perror("mmap"),exit(-1);
    }
    //2.使用映射的内存地址
    int* pi = p;
    *pi = 100;
    printf("*pi = %d\n",*pi);
    //3.删除映射
    int res = munmap(p,4);
    if(-1 == res)
    {
        perror("munmap"),exit(-1);
    }
    printf("解除映射成功\n");
    return 0;
}
