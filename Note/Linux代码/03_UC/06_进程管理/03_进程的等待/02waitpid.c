//waitpid函数的使用,等待指定进程结束
//根据下面三个不同选项,观察不同结果

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>

int main(void)
{
    //创建二个子进程
    pid_t pid1,pid2;
    pid1 = fork();
    if(-1 == pid1){
        perror("fork"),exit(-1);
    }
    if(0 != pid1){  //父进程
        pid2 = fork();
    }
    //三个进程：1个父进程 + 2个子进程
    if(0 == pid1){  //子进程一
        printf("子进程一%d开始运行\n",getpid());
        sleep(5);
        printf("子进程一终止\n");
        exit(100);
    }
    if(0 == pid2){  //子进程二
        printf("子进程二%d开始运行\n",getpid());
        sleep(10);
        printf("子进程二终止\n");
        exit(200);
    }
    printf("父进程开始等待...\n");
    int status;
    //int res = waitpid(-1,&status,WNOHANG);  //没有子进程退出立即返回
    //int res = waitpid(-1,&status,0);
    int res = waitpid(pid2,&status,0);
    if(-1 == res){
        perror("waitpid"),exit(-1);
    }
    printf("父进程等待结束\n");
    //判断进程是否正常终止
    if(WIFEXITED(status)){
        printf("等到的子进程是：%d,子进程的退出码是：%d\n",res,WEXITSTATUS(status));
    }
    return 0;
}
