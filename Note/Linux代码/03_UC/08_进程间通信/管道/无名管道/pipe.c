//无名管道通信演示，及pipe函数使用
//管道单向通信
#include <stdio.h>
#include <unistd.h>
#include <strings.h>

int main(void){
    int pfd[2];
    pid_t pid;
    char buf[12] = {0};
    //创建管道
    pipe(pfd);
    //创建子进程
    pid=fork();
    if(pid < 0){
        perror("fork");
        return 1;
    }
    //子进程写端，关闭读端
    if(pid==0){
        close(pfd[0]);
        sleep(5);
        write(pfd[1],"hello\n",6);
    }
    //父进程读端，关闭写端
    else {
        close(pfd[1]);
        read(pfd[0],buf,6); //读写函数自带阻塞效果
        //从显示器输出
        write(1,buf,6);
    }
    return 0;
}
