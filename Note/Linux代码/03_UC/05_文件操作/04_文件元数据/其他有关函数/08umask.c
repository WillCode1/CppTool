//umask函数的使用
//1.用于设置系统当前的屏蔽权限
//2.修改只对之后的文件有效
//3.返回值是先前的屏蔽权限

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

int main(void)
{
    mode_t old = umask(0055);
    //当前系统默认屏蔽 0002
    printf("old = %o\n",old);//2
    //创建一个新文件
    int fd = open("b.txt",O_RDWR|O_CREAT,0777);
    if(-1 == fd)
    {
        perror("open"),exit(-1);
    }
    //恢复系统默认的屏蔽字
    //对已经创建过的文件权限没有影响
    umask(old);
    close(fd);
    return 0;
}
