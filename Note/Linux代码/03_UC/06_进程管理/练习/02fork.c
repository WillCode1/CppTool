//使用函数递归创建父子4个进程
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>
#include <wait.h>
#include <stdlib.h>

int main(){
    pid_t pid[3];
    recurfork(pid, 3);
#if 1
    if((pid[0]<pid[1])&&(pid[1]<pid[2])){    //1
        printf("我是第父代进程,进程号是：%d\n", getpid());
    }
    else if(!pid[0]){   //2
        printf("我是第1个子进程,进程号是：%d\n", getpid());
    }
    else if(!pid[1]){   //3
        printf("我是第2个子进程,进程号是：%d\n", getpid());
    }
    else if(!pid[2]){   //4
        printf("我是第3个子进程,进程号是：%d\n", getpid());
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
        if(pid[cnt]){
            cnt++;
            recurfork(pid, size);
        }
    }
    return 0;
}
