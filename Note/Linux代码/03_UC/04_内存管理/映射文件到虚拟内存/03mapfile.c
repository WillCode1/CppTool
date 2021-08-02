//将文件映射到虚拟内存，可以提升操作文件的效率
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>

int main(void){
    int fd;
    //读写方式打开文件
    fd=open("hello",O_RDWR);
    if(fd==-1){
        perror("open");
        return 1;
    }
    //将文件映射到虚拟地址空间里
    void *p=mmap(NULL, 6, PROT_READ|PROT_WRITE, MAP_SHARED,fd,0);
    if(p==MAP_FAILED){
        perror("mmap");
        return 2;
    }
    int *str=(int *)p;
    str[0]=0x30313233;//如果结果顺序相反，大小端
    munmap(p,6);
    close(fd);
    return 0;
}
