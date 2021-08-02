//使用pthread_create函数创建新线程
//使用pthread_self函数获得线程tid
#include <stdio.h>
#include <pthread.h>

//线程任务函数
void *task(void *str){
    printf("%s",(char *)str);
    printf("thread id %lu\n",pthread_self());
}
int main(void){
    pthread_t tid;
    //创建一个线程
    pthread_create(&tid,NULL,task,"new:");
    usleep(1);
    task("main:");
    return 0;
}
