//线程的等待
//函数pthread_join带回的返回值
#include <stdio.h>
#include <pthread.h>

void *task(void *arg){
    static int status = 1;
    printf("arg is %s\n",(char *)arg);
    pthread_exit(&status);
}
	
int main(void){
    pthread_t tid;
    int *ret;
    pthread_create(&tid,NULL,task,"A");
    pthread_join(tid,(void **)&ret);
    printf("ret value %d\n",*ret);
    return 0;
}
