//fork创建的子进程可以继承父进程的信号处理方式
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <sys/types.h>

void fa(int signo)
{
    printf("捕获到了信号%d\n",signo);
}

int main(void)
{
    signal(SIGINT,fa);
    //signal(SIGQUIT,SIG_IGN);
    signal(SIGQUIT,SIG_DFL);
    //创建子进程
    pid_t pid = fork();
    if(-1 == pid)
    {
        perror("fork"),exit(-1);
    }
    if(0 == pid) //子进程
    {
        printf("子进程%d开始执行\n",getpid());
        while(1);
    }
    printf("父进程结束\n");
    return 0;
}
