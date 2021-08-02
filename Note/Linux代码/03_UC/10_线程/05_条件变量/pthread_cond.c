//使用链表实现生产者和消费者模型
#include <stdio.h>
#include <pthread.h>
#include <time.h>
#include <stdlib.h>

typedef struct msg{
    struct msg *next;
    int num;
}msg_t;

//全局变量编译时，自动初始化为0
msg_t *head;
//静态初始化互斥锁
pthread_mutex_t lock=PTHREAD_MUTEX_INITIALIZER;
//静态初始化条件变量
pthread_cond_t condition=PTHREAD_COND_INITIALIZER;

//生产者函数的实现
void *producer(void *mp){
    msg_t *p;
    while(1){
        p=malloc(sizeof(msg_t));
        p->num=rand()%1000+1;
        printf("生产了%d\n",p->num);
        //操作全局变量时,加上互斥锁
        pthread_mutex_lock(&lock);
        //生产出结构体
        p->next=head;
        head=p;
        pthread_mutex_unlock(&lock);
        //给等待条件的消费者发送信号
        pthread_cond_signal(&condition);
        sleep(rand()%5);
    }
}
//消费者函数的实现
void *consumer(void *mp){
    msg_t *p;
    while(1){
        //加上互斥锁
        pthread_mutex_lock(&lock);
        //如果没有生产，就一直等着生产
        if(head==NULL){
            pthread_cond_wait(&condition,&lock);
        }
        //消费掉结构体
        p=head;
        head=p->next;
        pthread_mutex_unlock(&lock);
        printf("消费了%d\n",p->num);
        free(p);
        sleep(rand()%5);
    }
}
int main(void){
    pthread_t pid;//生产者的tid
    pthread_t cid;//消费者的tid
    srand(time(NULL));
    //创建两个线程
    pthread_create(&pid,NULL,producer,NULL);
    pthread_create(&cid,NULL,consumer,NULL);
    pthread_join(pid,NULL);
    pthread_join(cid,NULL);
    //销毁互斥锁和条件变量
    pthread_mutex_destroy(&lock);
    pthread_cond_destroy(&condition);
    return 0;
}
