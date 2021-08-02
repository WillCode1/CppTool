//execl函数的使用,
//加载执行一个可执行文件

#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>

int main(void){
    pid_t pid;
    int status;
    pid=fork();
    if(pid<0){
        perror("fork");
        return 1;
    }
    if(pid==0){
        printf("child pid %d\n",getpid());
        //execl("/bin/aaa","aaa",NULL);
        execl("/bin/ps","ps","-o","pid,ppid,comm",NULL);
        //上一条语句错误的时候，才执行下边的语句,否则该进程结束
        execl("/bin/ls","ls","-l",NULL);
    }else{
        wait(&status);
    }
    return 0;
}
