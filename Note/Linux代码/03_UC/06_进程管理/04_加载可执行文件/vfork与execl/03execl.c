//vfork函数和execl函数的使用
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>

int main(void)
{
    //1.使用vfork函数创建子进程
    pid_t pid = vfork();
    if(-1 == pid)
    {
        perror("vfork"),exit(-1);
    }
    //2.子进程调用execl函数跳转
    if(0 == pid) //子进程
    {
        printf("子进程%d开始运行\n",getpid());
        sleep(5);
        //调用execl函数进行跳转
        execl("/bin/ls","ls","-l",NULL);
        //执行不到
        printf("子进程结束\n");
    }
    //3.父进程执行
    printf("父进程%d开始执行\n",getpid());
    printf("父进程结束\n");
    return 0;
}
