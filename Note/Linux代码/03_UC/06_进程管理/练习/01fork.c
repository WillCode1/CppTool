//使用函数递归创建祖孙4代进程
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>
#include <wait.h>
#include <stdlib.h>

int main(){
    pid_t pid[3] = {0};
    recurfork(pid, 3);
#if 1
    if(pid[0] != 0){    //1
        printf("我是第1代进程,进程号是：%d\n", getpid());
    }
    else if(pid[1] != 0){   //2
        printf("我是第2代进程,进程号是：%d\n", getpid());
    }
    else if(pid[2] != 0){   //3
        printf("我是第3代进程,进程号是：%d\n", getpid());
    }
    else if((!pid[0])&&(!pid[1])&&(!pid[2])){   //4
        printf("我是第4代进程,进程号是：%d\n", getpid());
    }
#endif
    return 0;
}

int recurfork(int pid[], int size){
    static int cnt = 0;
    if(cnt < size){
        pid[cnt] = fork();
        if(-1 == pid[cnt]){
            perror("fork"), exit(-1);
        }
        if(0 == pid[cnt]){
            cnt++;
            recurfork(pid, size);
        }
    }
    return 0;
}
