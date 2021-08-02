//使用fork函数创建子进程
//测试两个位置的sleep,观察效果
#include <stdlib.h>
#include <sys/types.h>
#include <stdio.h>
#include <unistd.h>

int main(){
    int num = 0;
    pid_t pid = fork();
    if(-1 == pid){
        perror("fork"), exit(-1);
    }
    if(!pid){
        //sleep(2);
        printf("我是子进程%d,我的父进程是：%d\n", getpid(), getppid());
    }
#if 1
    else {
        sleep(2);
        printf("我是父进程%d,我的子进程是：%d\n", getpid(), pid);
        //getchar();
    }
#endif
    //父子进程使用不同存储区
    printf("%d, %d\n", ++num, getpid());
    return 0;
}
