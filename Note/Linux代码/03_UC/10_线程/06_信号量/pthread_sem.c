//信号量实质是计数器
//使用环形队列实现生产者和消费者模型
#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>
#include <time.h>

int queue[5];
//定义两个信号量，表示货物和订单的计数
sem_t output,demand;

//生产者的线程
void *producer(void *arg){
    int p=0;
    while(1){
        //订单计数减1
        sem_wait(&demand);
        queue[p]=rand()%1000+1;
        printf("producer %d\n",queue[p]);
        //货物计数加1
        sem_post(&output);
        p=(p+1)%5;
        sleep(rand()%5);
    }
}
//消费者的线程
void *consumer(void *arg){
    int c=0;
    while(1){
        //货物计数减1
        sem_wait(&output);
        printf("consumer %d\n",queue[c]);
        queue[c]=0;
        //订单计数加1
        sem_post(&demand);
        c=(c+1)%5;
        sleep(rand()%5);
    }
}

int main(void){
    pthread_t ptid,ctid;
    srand(time(NULL));
    //初始化两个信号量，分别表示货物和订单的计数
    sem_init(&output,0,0);
    sem_init(&demand,0,5);
    //创建生产者和消费者两个线程
    pthread_create(&ptid,NULL,producer,NULL);
    pthread_create(&ctid,NULL,consumer,NULL);
    pthread_join(ptid,NULL);
    pthread_join(ctid,NULL);
    //销毁两个信号量
    sem_destroy(&demand);
    sem_destroy(&output);
    return 0;
}
