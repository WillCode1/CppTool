//使用wait函数等待子进程结束
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>

int main(void)
{
    //1.创建子进程
    pid_t pid = fork();
    if(-1 == pid)
    {
        perror("fork"),exit(-1);
    }
    //2.子进程工作10秒后终止
    if(0 == pid) //子进程
    {
        printf("child %d start\n",getpid());
        sleep(5);
        printf("child end\n");
        _exit(37);
    }
    //3.父进程等待子进程结束
    printf("parent wait\n");
    int status = 0;
    int res = wait(&status);
    printf("wait end, status: %d,exit child pid: %d\n",status,res);
    //判断子进程是否正常终止
    if(WIFEXITED(status))
    {
        //获取子进程退出状态信息
        printf("child exit status: %d\n",WEXITSTATUS(status));
    }
    return 0;
}
