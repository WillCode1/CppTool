//使用F_GETLK参数判断锁能否加上
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
    lock.l_whence = SEEK_SET;
    lock.l_start = 0;
    lock.l_len = 5;
    lock.l_pid = -1;
    //3.测试锁能否加上
    int res = fcntl(fd,F_GETLK,&lock);
    if(-1 == res)
    {
        perror("fcntl"),exit(-1);
    }
    printf("F_RDLCK = %d,F_WRLCK = %d,F_UNLCK = %d\n",F_RDLCK,F_WRLCK,F_UNLCK);
    printf("lock.l_type = %d,lock.l_pid = %d\n",lock.l_type,lock.l_pid);
    
    if(-1 == lock.l_pid)
    {
        printf("可以加锁\n");
    }
    else
    {
        printf("不可以加锁\n");
    }
    //4.关闭文件
    close(fd);
    return 0;
}
