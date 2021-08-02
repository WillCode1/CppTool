//open函数的第二个参数的本质
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

int main(void)
{
    //1.打开/创建文件
    //文件存在则清空文件
    int fd = open("a.txt",O_WRONLY|O_CREAT|O_TRUNC,0644);
    if(-1 == fd)
    {
        perror("open");
        return -1;
    }
    // 1 64 512 进行按位或则对应位置1
    printf("O_WRONLY = %d,O_CREAT = %d,O_TRUNC = %d\n",O_WRONLY,O_CREAT,O_TRUNC);

    //2.关闭文件
    close(fd);
    return 0;
}
