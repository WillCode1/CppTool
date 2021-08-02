//使用kill函数发送信号
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
    pid_t pid = fork();
    if(-1 == pid)
    {
        perror("fork"),exit(-1);
    }
    if(0 == pid) //子进程
    {
        printf("pid = %d\n",getpid());
        signal(50,fa);
        while(1);
    }
    sleep(1);
    //判断子进程是否存在
    if(0 == kill(pid,0))
    {
        printf("父进程发送信号50\n");
        kill(pid,50);
    }
    return 0;
}
