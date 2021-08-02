//此方法仅供娱乐
#include <stdio.h>
#include <pthread.h>

void *task(void *arg){
    int status = 1;
    //仅供娱乐
    pthread_exit((void *)status);
}
	
int main(void){
    pthread_t tid;
    int ret;
    pthread_create(&tid,NULL,task,NULL);
    pthread_join(tid,(void **)&ret);
    printf("ret value %d\n",ret);
    return 0;
}
