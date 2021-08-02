//多线程的异步访问
#include <stdio.h>
#include <pthread.h>

int count=0;

void *task(void *arg){
    int val, i;
    for(i=0;i<5000;i++){
        val=count;
    	printf("thread: %lu..%d\n",pthread_self(),val+1);
    	count=val+1;
    }
    return NULL;
}

int main(void){
    pthread_t tidA,tidB;
    //创建两个线程
    pthread_create(&tidA,NULL,task,NULL);
    pthread_create(&tidB,NULL,task,NULL);
    pthread_join(tidA,NULL);
    pthread_join(tidB,NULL);
    return 0;
}
