//使用顺序存储结构实现堆栈的基本操作
#include <stdio.h>

//给数据类型起个别名,支持其他数据类型
//typedef int datatype;
//通过宏来指定数组的大小,便于修改
//#define SIZE 50

//定义堆栈的数据类型
typedef struct
{
    //datatype arr[SIZE];
    int arr[5];//存储具体的元素值
    int pos;//记录数组的下标
}Stack;

//自定义函数实现入栈操作
void push(Stack* ps,int data);
//自定义函数实现遍历操作
void travel(Stack* ps);
//自定义函数实现出栈操作
int pop(Stack* ps);
//查看栈顶元素
int peek(Stack* ps);
//判断栈是否为满
int full(Stack* ps);
//判断栈是否为空
int empty(Stack* ps);
//计算栈中元素的个数
int size(Stack* ps);

int main(void)
{
    Stack stack;//创建一个堆栈
    stack.pos = 0;
    int i = 0;
    for(i = 0; i < 6; i++)
    {
        push(&stack,i);
    }
    travel(&stack); //0 1 2 3 4
    printf("出栈的元素是：%d\n",pop(&stack)); //4
    travel(&stack); //0 1 2 3 

    printf("--------------------\n");
    printf("栈顶元素是：%d\n",peek(&stack));
    printf("%s\n",full(&stack)?"堆栈已经满了":"堆栈没有满");
    printf("%s\n",empty(&stack)?"堆栈空了":"堆栈没有空");
    printf("堆栈中的元素个数是：%d\n",size(&stack));
    return 0;
}

//查看栈顶元素
int peek(Stack* ps)
{
    //判断是否为空
    //if(0 == ps->pos)
    if(empty(ps))
    {
        printf("堆栈为空,查看栈顶元素失败\n");
        return -1;
    }
    return ps->arr[ps->pos-1];
}

//判断栈是否为满
int full(Stack* ps)
{
    return 5 == ps->pos;
}
//判断栈是否为空
int empty(Stack* ps)
{
    return 0 == ps->pos;
}

//计算栈中元素的个数
int size(Stack* ps)
{
    return ps->pos;
}

//自定义函数实现出栈操作
int pop(Stack* ps)
{
    //判断堆栈是否为空
    if(empty(ps))
    {
        printf("堆栈已经空了，出栈失败\n");
        return -1;//表示出错
    }
    //return ps->arr[ps->pos-1];
    //ps->pos--;
    return ps->arr[--ps->pos];
}

//自定义函数实现遍历操作
void travel(Stack* ps)
{
    printf("栈中的元素有：");
    int i = 0;
    for(i = 0; i < ps->pos; i++)
    {
        printf("%d ",ps->arr[i]);
    }
    printf("\n");
}

//自定义函数实现入栈操作
void push(Stack* ps,int data)
{
    //判断是否为满
    //if(5 == ps->pos)
    if(full(ps))
    {
        printf("栈已经满了,入栈失败\n");
        return;
    }
    ps->arr[ps->pos++] = data;
    //ps->pos++;
}
