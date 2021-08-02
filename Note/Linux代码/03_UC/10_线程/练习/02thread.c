//pthread_create函数的使用
//启动1个子线程，打印1~100之间数
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
void* task(void* p)
{
    int i = 0;
    for(i = 0; i < 100; i++)
    {
        printf("子线程中：i = %d\n",i);
    }
}

int main(void)
{
    pthread_t tid;
    pthread_create(&tid,NULL,task,NULL);
    //usleep(100000);
    int i = 0;
    for(i = 0; i < 100; i++)
    {
        printf("主线程中：i = %d\n",i);
    }
    printf("子线程ID是：%lu\n",tid);
    printf("主线程ID是：%lu\n",pthread_self());
    return 0;
}
