//系统调用open函数和close函数的使用
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

int main(void)
{
    //1.打开/创建文件
    //(1)读写文件，文件必须存在
    //int fd=open("hello",O_RDWR);
    //(1)文件不存在则创建，存在则打开
    int fd = open("a.txt",O_RDONLY|O_CREAT,0644);
    //(2)文件不存在则创建,存在则创建失败
    //int fd = open("a.txt",O_RDONLY|O_CREAT|O_EXCL,0644);
    //(3)文件存在则清空文件
    //int fd = open("a.txt",O_WRONLY|O_CREAT|O_TRUNC,0644);
    if(-1 == fd)
    {
        perror("open");
        return -1;
    }
    printf("打开文件成功\n");
    //2.关闭文件
    int res = close(fd);
    if(-1 == res)
    {
        perror("close");
        return -1;
    }
    printf("关闭文件成功\n");
    return 0;
}
