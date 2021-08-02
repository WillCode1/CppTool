//基于链式结构的队列操作
#include <stdio.h>
#include <stdlib.h>

//定义节点的数据类型
typedef struct Node
{
    int data;//存放具体的数据内容
    struct Node* next;//下一个节点地址
}Node;

//定义队列的数据类型
typedef struct
{
    Node* head;//保存第一个节点的地址
}Queue;

//入队操作
void push(Queue* pq,int data);
//遍历操作
void travel(Queue* pq);
//判断队列是否为空
int empty(Queue* pq);
//判断队列是否为满
int full(Queue* pq);
//获取队首元素的值
int get_head(Queue* pq);
//获取队尾元素的值
int get_tail(Queue* pq);
//计算队列中节点的个数
int size(Queue* pq);
//出队操作
int pop(Queue* pq);
//清空队列中所有的元素
void clear(Queue* pq);

int main(void)
{
    //1.创建队列并且进行初始化
    Queue queue;
    queue.head = NULL;
    push(&queue,11);
    travel(&queue); //11
    push(&queue,22);
    travel(&queue); //11 22
    push(&queue,33);
    travel(&queue); //11 22 33

    printf("------------------\n");
    printf("%s\n",empty(&queue)?"队列已经空了":"队列没有空");
    printf("%s\n",full(&queue)?"队列已经满了":"队列没有满");
    printf("队首元素是：%d\n",get_head(&queue));
    printf("队尾元素是：%d\n",get_tail(&queue));
    printf("队列中的元素个数是：%d\n",size(&queue));

    printf("---------------------\n");
    travel(&queue);
    printf("出队的元素是：%d\n",pop(&queue));
    clear(&queue);
    travel(&queue);
    return 0;
}

//出队操作
int pop(Queue* pq)
{
    if(empty(pq))
    {
        return -1;//表示出错
    }
    //保存第一个节点的地址
    Node* p = pq->head;
    //头指针指向第二个节点
    pq->head = pq->head->next;
    //保存要删除的节点数据
    int temp = p->data;
    //删除第一个节点
    free(p);
    p = NULL;
    return temp;

}
//清空队列中所有的元素
void clear(Queue* pq)
{
    while(pq->head != NULL)
    {
        //保存第一个节点地址
        Node* p = pq->head;
        //头指针指向下一个节点
        pq->head = pq->head->next;
        //释放第一个节点
        free(p);
        p = NULL;
    }
}
//判断队列是否为空
int empty(Queue* pq)
{
    return NULL == pq->head;
}
//判断队列是否为满
int full(Queue* pq)
{
    return 0;
}
//获取队首元素的值
int get_head(Queue* pq)
{
    if(empty(pq))
    {
        return -1;//表示出错
    }
    return pq->head->data;
}
//获取队尾元素的值
int get_tail(Queue* pq)
{
    if(empty(pq))
    {
        return -1;//表示出错
    }
    Node* p = pq->head;
    while(p->next != NULL)
    {
        //指向下一个节点
        p = p->next;
    }
    return p->data;
}
//计算队列中节点的个数
int size(Queue* pq)
{
    int count = 0;
    Node* p = pq->head;
    while(p != NULL)
    {
        count++;
        p = p->next;
    }
    return count;
}
//遍历操作
void travel(Queue* pq)
{
    printf("队列中的元素有：");
    Node* p = pq->head;
    while(p != NULL)
    {
        printf("%d ",p->data);
        //指向下一个
        p = p->next;
    }
    printf("\n");
}

//入队操作
void push(Queue* pq,int data)
{
    //1.创建新节点
    Node* pn = (Node*)malloc(sizeof(Node));
    pn->data = data;
    pn->next = NULL;
    //2.挂接新节点到队列的尾部
    //2.1 如果队列为空，则直接连接
    if(NULL == pq->head)
    {
        pq->head = pn;
    }
    //2.2 如果队列不为空，使用尾节点连接新节点
    else
    {
        Node* p = pq->head;
        // p指向的节点不是尾节点,则寻找下一个节点 再比较
        while(p->next != NULL)
        {
            //指向下一个节点
            p = p->next;
        }
        //使用尾节点连接新节点
        p->next = pn;
    }
}


