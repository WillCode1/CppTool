//使用信号量集实现进程间的通信
#include <stdio.h>
#include <sys/types.h>
#include <sys/sem.h>

int main(void){
    //获取键值
    key_t key=ftok(".",41);
    if(key==-1){
        perror("ftok");
        return 1;
    }
    //获取信号量集的id
    int semid=semget(key,1,0);
    if(semid==-1){
        perror("semget");
        return 2;
    }
    while(1){
        //操作信号量集中某个信号
        int semret=semctl(semid,0,GETVAL);
        if(semret==-1){
            perror("semctl");
            return 3;
        }
        if(semret>0){
            printf("%d resources could be used\n",semret);
        }
        else{
            printf("no resource could be used\n");
            break;
        }
        sleep(3);
    }
    return 0;
}
