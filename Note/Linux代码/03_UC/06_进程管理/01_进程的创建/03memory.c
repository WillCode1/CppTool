//父子进程之间的内存资源不共享(代码区除外)
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>

int i1 = 1;//全局变量  全局区

int main(void)
{
    int i2 = 1;//局部变量 栈区
    //st 指向堆区
    char* st = (char*)malloc(20);
    strcpy(st,"hello");
    //创建子进程
    pid_t pid = fork();
    if(-1 == pid)
    {
        perror("fork"),exit(-1);
    }
    if(0 == pid) //子进程
    {
        i1 = 2;
        i2 = 2;
        st[0] = 'H';
        int i3 = 2;
        printf("子进程%d:i1 = %d,i2 = %d,st = %s,i3 = %d\n",getpid(),i1,i2,st,i3);
        exit(0);//终止子进程
    }
    printf("父进程%d:i1 = %d,i2 = %d,st = %s\n",getpid(),i1,i2,st);
    return 0;
}
