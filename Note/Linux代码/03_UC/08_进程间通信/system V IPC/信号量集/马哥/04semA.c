//使用信号量集实现进程间的通信
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include <signal.h>

//定义全局变量保存信号量集的ID
int semid;
//版本更新,这个联合头文件中已经没有了
//需要自己定义
union semun {
    int val;  
    struct semid_ds *buf;
    unsigned short *array;
};

void fa(int signo){
    int res = semctl(semid,0,IPC_RMID);
    if(-1 == res){
        perror("semctl"),exit(-1);
    }
    printf("删除信号量集成功\n");
    exit(0);
}

int main(void){
    //1.获取key值,使用ftok函数
    key_t key = ftok(".",200);
    if(-1 == key){
        perror("ftok"),exit(-1);
    }
    //2.创建信号量集，使用semget函数
    semid = semget(key,1,IPC_CREAT|IPC_EXCL|0644);
    if(-1 == semid){
        perror("semget"),exit(-1);
    }
    //3.初始化信号量集,使用semctl函数
    union semun semopts;
    semopts.val = 5;
    int res = semctl(semid,0,SETVAL,semopts);
    if(-1 == res){
        perror("semctl"),exit(-1);
    }
    //4.删除信号量集，使用信号2处理
    printf("删除信号量集，请按ctrl+c...\n");
    signal(2,fa);
    while(1);
    return 0;
}
