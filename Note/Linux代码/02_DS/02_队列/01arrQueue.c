//采用顺序存储结构实现队列的操作
#include <stdio.h>

//定义队列的数据类型
typedef struct
{
    int arr[5];//存储元素的位置
    int front;//记录队首元素的下标
    int rear;//下一个可以存放元素下标
}Queue;

//入队操作 值传递 址传递 
void push(Queue* pq,int data);
//遍历队列中的所有元素
void travel(Queue* pq);
//出队操作
int pop(Queue* pq);
//判断队列是否为空
int empty(Queue* pq);
//判断队列是否为满
int full(Queue* pq);
//查看队首元素
int get_head(Queue* pq);
//查看队尾元素
int get_tail(Queue* pq);
//计算队列中元素个数
int size(Queue* pq);

int main(void)
{
    //创建队列，并且进行初始化
    Queue queue;
    queue.front = 0;
    queue.rear = 0;

    push(&queue,11);
    travel(&queue); //11
    push(&queue,22);
    travel(&queue); //11 22
    push(&queue,33);
    travel(&queue); //11 22 33
    
    printf("出队的元素是：%d\n",pop(&queue));
    travel(&queue); //22 33

    printf("------------------\n");
    printf("队首元素是：%d\n",get_head(&queue));
    printf("队尾元素是：%d\n",get_tail(&queue));
    printf("队列中元素个数是：%d\n",size(&queue));
    printf("%s\n",empty(&queue)?"队列已经空了":"队列没有空");
    printf("%s\n",full(&queue)?"队列已经满了":"队列没有满");
    return 0;
}

//判断队列是否为空
int empty(Queue* pq)
{
    return 0 == pq->rear-pq->front;
}

//判断队列是否为满
int full(Queue* pq)
{
    return 5 == pq->rear-pq->front;
}
//查看队首元素
int get_head(Queue* pq)
{
    //判断队列是否为空
    if(empty(pq))
    {
        return -1;//代表出错
    }
    return pq->arr[pq->front%5];
}
//查看队尾元素
int get_tail(Queue* pq)
{
    //判断队列是否为空
    if(empty(pq))
    {
        return -1;//表示出错
    }
    return pq->arr[(pq->rear-1)%5];
}
//计算队列中元素个数
int size(Queue* pq)
{
    return pq->rear-pq->front;
}
//出队操作
int pop(Queue* pq)
{
    //判断队列是否为空
    if(empty(pq))
    {
        return -1;//表示出错
    }
    int temp = pq->arr[pq->front%5];
    pq->front++;
    return temp;
}

//遍历队列中的所有元素
void travel(Queue* pq)
{
    printf("队列中的元素有：");
    int i = 0;
    for(i = pq->front; i < pq->rear;i++)
    {
        printf("%d ",pq->arr[i%5]);
    }
    printf("\n");
}

//入队操作 值传递 址传递 
void push(Queue* pq,int data)
{
    //判断队列为满的情况
    if(full(pq))
    {
        return; //表示出错
    }
    pq->arr[pq->rear%5] = data; 
    pq->rear++;
}


