//sigqueue函数和sigaction函数的使用
//给指定进程发送指定信号和附加数据
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <signal.h>

void fa(int signo,siginfo_t* info,void* p)
{
    printf("进程%d发送来了信号%d,附加数据是：%d\n",info->si_pid,signo,info->si_value);
}

int main(void)
{
    //设置信号捕获函数sigqueue
    struct sigaction action = {};
    //初始化第二个函数指针进行处理
    action.sa_sigaction = fa;
    //采用结构体中第二个函数指针处理
    action.sa_flags = SA_SIGINFO;
    sigaction(40,&action,NULL);
    
    //创建子进程给父进程发信号和数据
    pid_t pid = fork();
    if(-1 == pid){
        perror("fork"),exit(-1);
    }
    if(0 == pid){   //子进程
        int i = 0;
        //准备sigqueue函数要发送的数据
        union sigval v;
        //发送信号和附加数据
        for(i = 0; i < 100; i++){
            v.sival_int = i;
            sigqueue(getppid(),40,v);
        }
        sleep(1);
        _exit(100);//终止子进程
    }

    //父进程等待处理信号和附加数据
    while(1);
    return 0;
}
