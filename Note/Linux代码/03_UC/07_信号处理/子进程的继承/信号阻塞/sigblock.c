/*
 * 信号集屏蔽的继承
 * 发送kill -2给子进程,证明以上结论
 */
#include <stdio.h>
#include <signal.h>
#include <stdlib.h>

int main(void){
    sigset_t set, oldset, pset;
    int res, pid;
    //将集合清空
    sigemptyset(&set);
    //将2号信号添加到集合set里
    sigaddset(&set,SIGINT);
    //设置信号阻塞掩码
    sigprocmask(SIG_BLOCK,&set,&oldset);
    pid = fork();
    if(-1 == pid){
        perror("fork"), exit(-1);
    }
    if(0 == pid){
        while(1){
            sigpending(&pset);
            res = sigismember(&pset,SIGINT);
            if(res == 0){
                printf("not pending...\n");
            }else{
                printf("SIGINT pending...\n");
            }
        }
    }
    else {  //父进程
        wait(NULL);
    }
    return 0;
}
