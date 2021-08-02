//vfork函数的使用
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>

int main(void)
{
    //使用vfork函数创建子进程
    pid_t pid = vfork();
    if(-1 == pid)
    {
        perror("vfork"),exit(-1);
    }
    if(0 == pid) //子进程
    {
        printf("子进程%d开始运行\n",getpid());
        sleep(5);
        printf("子进程结束\n");
        //子进程不退出，则结果不可预知
        _exit(0);//终止子进程
    }
    printf("父进程%d开始执行\n",getpid());
    printf("父进程结束\n");
    return 0;
}
