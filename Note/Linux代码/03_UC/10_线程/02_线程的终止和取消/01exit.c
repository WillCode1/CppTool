//线程的终止
//return,exit和pthread_exit的区别

#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>

//线程任务函数
void *task(void *p){
    printf("process id: %d\n",getpid());
    printf("%s tid: %lu\n", (char *)p, pthread_self());
    pthread_exit(NULL);
    //exit(1);  //进程结束
    //return;   //线程任务函数结束,线程没有消失
}

int main(){
    pthread_t tid;
    //创建一个线程
    pthread_create(&tid,NULL,task,"new:");
    usleep(1);
    task("main:");
    printf("new tid: %lu\n", tid);
    return 0;
}
