//线程取消函数的使用
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>

void* task(void* p){
    //设置不允许被取消
    pthread_setcancelstate(PTHREAD_CANCEL_DISABLE,NULL);
    //设置为立即取消
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS,NULL);
    while(1){
        printf("I am superman!\n");
        usleep(100000);
    }
}

void* task2(void* p){
    printf("开始取消线程...\n");
    usleep(300000);
    printf("取消线程结束\n");
    pthread_cancel(*(pthread_t*)p);
}

int main(void){
    //1.启动一个新线程，不断进行打印
    pthread_t tid;
    pthread_create(&tid,NULL,task,NULL);
    //2.启动另外一个新线程,负责取消上述线程
    pthread_t tid2;
    pthread_create(&tid2,NULL,task2,&tid);
    //3.主线程等待子线程的结束
    pthread_join(tid,NULL);
    pthread_join(tid2,NULL);
    return 0;
}
