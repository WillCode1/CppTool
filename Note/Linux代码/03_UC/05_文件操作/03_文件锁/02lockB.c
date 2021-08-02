//使用fcntl函数对文件进行再次加锁
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

int main(void)
{
    //1.打开文件
    int fd = open("a.txt",O_RDWR);
    if(-1 == fd)
    {
        perror("open"),exit(-1);
    }
    //2.准备一把锁
    struct flock lock;
    lock.l_type = F_RDLCK;
    //lock.l_type = F_WRLCK;
    lock.l_whence = SEEK_SET;
    lock.l_start = 0;
    lock.l_len = 10;
    lock.l_pid = -1;
    //3.使用fcntl函数加锁
    int res = fcntl(fd,F_SETLK,&lock);
    if(-1 == res)
    {
        printf("加锁失败");
    }
    else
    {
        printf("加锁成功\n");
    }
    //4.关闭文件
    close(fd);
    return 0;
}
