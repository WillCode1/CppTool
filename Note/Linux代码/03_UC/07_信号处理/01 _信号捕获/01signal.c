//使用signal函数设置信号的处理方式
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <sys/types.h>

//自定义信号处理函数
void handler(int signo)
{
    printf("捕获到了信号%d\n",signo);
}

int main(void)
{
    printf("pid = %d\n",getpid());
    //设置信号2进行忽略处理
    signal(2,SIG_IGN);
    //设置信号3进行自定义处理
    signal(SIGQUIT,handler);
    //设置信号9进行自定义处理
    //信号9专门用来杀死进程，只能进行默认处理
    signal(9,handler);
    while(1);
    return 0;
}
