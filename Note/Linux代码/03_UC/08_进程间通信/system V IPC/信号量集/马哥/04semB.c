//使用信号量集实现进程间的通信
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/sem.h>

int main(void){
    //1.获取key值,使用ftok函数
    key_t key = ftok(".",200);
    if(-1 == key){
        perror("ftok"),exit(-1);
    }
    //2.获取信号量集,使用semget函数
    int semid = semget(key,1,0);
    if(-1 == semid){
        perror("semget"),exit(-1);
    }
    //3.创建10个子进程模拟抢占资源
    int i = 0;
    for(i = 0; i < 10; i++){
        pid_t pid = fork();
        if(0 == pid){   //子进程
            //准备结构体变量
            struct sembuf op={0,-1,SEM_UNDO};
            //使用semop函数占用资源
            semop(semid,&op,1);
            printf("申请共享资源成功\n");
            sleep(5);
            op.sem_op = 1;
            //使用semop函数释放资源
            semop(semid,&op,1);
            printf("释放共享资源完毕\n");
            exit(0);
        }
    }
    return 0;
}
