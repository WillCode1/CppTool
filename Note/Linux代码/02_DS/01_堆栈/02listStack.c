//基于链式存储结构的堆栈实现
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

//定义节点的数据类型
typedef struct Node
{
    int data;//存放的具体元素
    struct Node* next;//存放下个节点地址
}Node;

//定义堆栈的数据类型
typedef struct
{
    Node* head;
}Stack;

//入栈操作
void push(Stack* ps,int data);
//遍历操作
void travel(Stack* ps);
//出栈操作
int pop(Stack* ps);
//查看栈顶元素
int peek(Stack* ps);
//判断堆栈是否为空
bool empty(Stack* ps);
//判断堆栈是否为满
bool full(Stack* ps);
//计算堆栈中的节点个数
int size(Stack* ps);

int main(void)
{
    //创建堆栈并且进行初始化
    Stack stack;
    stack.head = NULL;
    push(&stack,11);
    travel(&stack); //11
    push(&stack,22);
    travel(&stack); //22 11
    push(&stack,33);
    travel(&stack); //33 22 11
    printf("出栈的元素是：%d\n",pop(&stack));
    travel(&stack);

    printf("-------------------\n");
    printf("栈顶元素是：%d\n",peek(&stack));
    printf("%s\n",empty(&stack)?"堆栈为空":"堆栈不为空");
    printf("%s\n",full(&stack)?"堆栈为满":"堆栈没有满");
    printf("堆栈中的元素个数是：%d\n",size(&stack));
    return 0;
}

//查看栈顶元素
int peek(Stack* ps)
{
    //判断堆栈是否为空
    if(empty(ps))
    {
        return -1;//查看失败
        //(以后讲到,如果返回值也恰好是-1的情况)
    }
    return ps->head->data;
}
//判断堆栈是否为空
bool empty(Stack* ps)
{
    return NULL == ps->head;
}
//判断堆栈是否为满
bool full(Stack* ps)
{
    return false;
}
//计算堆栈中的节点个数
int size(Stack* ps)
{
    int count = 0;
    Node* p = ps->head;
    while(p != NULL)
    {
        count++;
        //指向下一个节点
        p = p->next;
    }
    return count;
}

//出栈操作
int pop(Stack* ps)
{
    //判断堆栈是否为空
    if(empty(ps))
    {
        return -1;//表示错误
    }
    //保存要删除节点的地址
    Node* p = ps->head;
    //头指针指向下一个节点
    ps->head = ps->head->next;
    //存储要删除节点的元素值
    int temp = p->data;
    free(p);
    p = NULL;
    return temp;
}

//遍历操作
void travel(Stack* ps)
{
    Node* p = ps->head;
    printf("堆栈中的元素有：");
    while(p != NULL)
    {
        printf("%d ",p->data);
        //指向下一个节点
        p = p->next;
    }
    printf("\n");
}

//入栈操作
void push(Stack* ps,int data)
{
    //1.创建新节点
    //Node node; error 生命周期短暂
    //Node* pn = &node;
    Node* pn = (Node*)malloc(sizeof(Node));
    pn->data = data;
    pn->next = NULL;
    //2.插入新节点
    pn->next = ps->head;
    ps->head = pn;
}

