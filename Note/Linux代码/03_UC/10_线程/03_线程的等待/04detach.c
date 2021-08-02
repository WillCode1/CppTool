//线程的分离
//pthread_detach函数
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>

void* task(void* p){
    //1.子线程中打印1~20之间的数
    int i = 0;
    for(i = 1; i <= 20; i++){
        printf("子线程中：i = %d\n",i);
    }
}

int main(void){
    pthread_t tid;
    pthread_create(&tid,NULL,task,NULL);
    //设置子线程为分离的状态
    pthread_detach(tid);
    //此时设置等待,但其实非阻塞且无效
    pthread_join(tid,NULL);
    int i = 0;
    //2.主线程中打印1~20之间的数
    for(i = 1; i <= 20; i++){
        printf("主线程中：i = %d\n",i);
    }
    return 0;
}
