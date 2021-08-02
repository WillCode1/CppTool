//观察vfork和execl创建子进程对信号的处理
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <signal.h>

void fa(int signo)
{
    printf("捕获到了信号%d\n",signo);
}

int main(void)
{
    signal(SIGINT,fa);
    signal(SIGQUIT,SIG_IGN);
    //使用vfork函数创建子进程
    pid_t pid = vfork();
    if(-1 == pid)
    {
        perror("vfork"),exit(-1);
    }
    if(0 == pid) //子进程
    {
        printf("子进程%d开始运行\n",getpid());
	//while(1);
        execl("./proc","proc",NULL);
    }
    wait(NULL);
    printf("父进程结束\n");
    return 0;
}
