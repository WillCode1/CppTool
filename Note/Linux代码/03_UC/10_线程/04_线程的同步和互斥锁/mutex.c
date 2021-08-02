//用互斥锁实现多线程同步
#include <stdio.h>
#include <pthread.h>

int count=0;
//静态初始化一个mutex锁
pthread_mutex_t mutex=PTHREAD_MUTEX_INITIALIZER;

void *task(void *arg){
    int val, i;
    for(i=0;i<5000;i++){
        //加锁
        pthread_mutex_lock(&mutex);
        val=count;
        printf("thread: %lu..%d\n",pthread_self(),val+1);
        count=val+1;
        //解锁
        pthread_mutex_unlock(&mutex);
    }
    return NULL;
}

int main(void){
    pthread_t tidA,tidB;
    //pthread_mutex_init(&mutex,0);
    //创建两个线程
    pthread_create(&tidA,NULL,task,NULL);
    pthread_create(&tidB,NULL,task,NULL);
    pthread_join(tidA,NULL);
    pthread_join(tidB,NULL);
    //销毁mutex锁
    pthread_mutex_destroy(&mutex);
    return 0;
}
