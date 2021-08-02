//使用信号量集实现进程间的通信
#include <stdio.h>
#include <sys/types.h>
#include <sys/sem.h>
#include <sys/ipc.h>
//版本更新,这个联合头文件中已经没有了
//需要自己定义
union semun {
    int val;  
    struct semid_ds *buf;
    unsigned short *array;
};
int main(void){
    //获取键值
    key_t key=ftok(".",41);
    if(key==-1){
        perror("ftok");
        return 1;
    }
    //获取键值对应的信号量集的id
    int semid=semget(key,1,IPC_CREAT|0664);
    if(semid==-1){
        perror("semget");
        return 2;
    }
    //初始化信号量集的第一个信号量
    union semun semopts;
    semopts.val=5;
    int semret=semctl(semid,0,SETVAL,semopts);
    if(semret==-1){
        perror("semctl");
        return 3;
    }
    //操作资源，每隔三秒减少一个资源
    struct sembuf op={0,-1,IPC_NOWAIT};
    while(1){
        int opret=semop(semid,&op,1);
        if(opret==-1){
            perror("semop");
            return 4;
        }
        sleep(3);
    }
    return 0;
}
