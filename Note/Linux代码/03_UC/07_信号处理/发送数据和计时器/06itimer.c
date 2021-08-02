//计时器的使用
//setitimer,getitimer
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/time.h>
#include <signal.h>

void fa(int signo)
{
    printf("过了2.3秒\n");
}

int main(void)
{
    signal(SIGALRM,fa);
    //准备一个计时器
    struct itimerval timer;
    //设置启动时间
    timer.it_value.tv_sec = 5;
    timer.it_value.tv_usec = 0;
    //设置间隔时间
    timer.it_interval.tv_sec = 2;
    timer.it_interval.tv_usec=300000;

    //设置真实计时器开始启动
    int res = setitimer(ITIMER_REAL,&timer,NULL);
    if(-1 == res)
    {
        perror("setitimer"),exit(-1);
    }
    while(1);
    return 0;
}
