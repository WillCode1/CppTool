//实现单链表中的各种操作
#include <stdio.h>
#include <stdlib.h>

//定义节点的数据类型
typedef struct Node
{
    int data;//数据内容,可以是其他的数据类型 
    struct Node* next;//下一个节点地址
}Node;

//定义链表的数据类型
typedef struct
{
    Node* head;//头指针
    int cnt;//[记录节点的个数]
}List;

//向链表的头节点位置插入新节点
void push_head(List* pl,int data);
//遍历链表的操作
void travel(List* pl);
//清空链表中的所有节点 clear
void clear(List* pl);
//计算链表中节点的个数 size
int size(List* pl);
//判断链表是否为空 empty
int empty(List* pl);
//判断链表是否为满 full
int full(List* pl);
//向指定的位置插入指定的新节点
void insert(List* pl,int pos,int data);
//创建新节点的功能函数
Node* create_node(int data);
//向链表的尾部插入新节点
void push_tail(List* pl,int data);
//删除链表中指定下标的节点
void del(List* pl,int pos);

int main(void)
{
    //创建链表，并且进行初始化
    List list;
    list.head = NULL;
    list.cnt = 0;
    //插入新节点，并且进行遍历
    push_head(&list,11);
    travel(&list); //11
    push_head(&list,22);
    travel(&list); //22 11
    push_head(&list,33);
    travel(&list); //33 22 11

    printf("---------------\n");
    printf("链表中节点的个数是：%d\n",size(&list));
    printf("%s\n",empty(&list)?"链表为空":"链表不为空");
    printf("%s\n",full(&list)?"链表为满":"链表不为满");
    printf("--------------------\n");
    travel(&list); //33 22 11
    insert(&list,-2,44);
    travel(&list); //33 22 11 44
    insert(&list,0,55);
    travel(&list); //55 33 22 11 44
    insert(&list,2,66);
    travel(&list);//55 33 66 22 11 44
    insert(&list,6,77);
    travel(&list);//77在最后
    printf("------------------\n");
    del(&list,-2); //删除失败
    del(&list,0);
    travel(&list);//33 66 22 11 44 77
    del(&list,1);
    travel(&list);//33 22 11 44 77
    del(&list,5);//删除失败
    travel(&list);//33 22 11 44 77
    del(&list,4);
    travel(&list);//33 22 11 44
    printf("-------------------\n");
    clear(&list);
    travel(&list);
    return 0;
}

//删除链表中指定下标的节点
void del(List* pl,int pos)
{
    //1.判断坐标是否合法
    if(pos < 0 || pos >= pl->cnt)
    {
        printf("坐标不合法，删除节点失败\n");
        return;
    }
    //2.当删除头节点时的处理方案
    if(0 == pos)
    {
        Node* p = pl->head;
        pl->head = pl->head->next;
        free(p);
        p = NULL;
        pl->cnt--;
        return;
    }
    //3.当删除其他节点时的处理方法
    Node* p = pl->head;
    int i = 0;
    for(i = 1; i < pos; i++)
    {
        p = p->next;
    }
    //下面的代码是pos=1时的功能代码
    Node* q = p->next;
    p->next = p->next->next;
    free(q);
    q = NULL;
    pl->cnt--;
}

//向链表的尾部插入新节点
void push_tail(List* pl,int data)
{
    insert(pl,pl->cnt,data);
}

//创建新节点的功能函数
Node* create_node(int data)
{
    Node* pn = (Node*)malloc(sizeof(Node));
    pn->data = data;
    pn->next = NULL;
    return pn;
}

//向指定的位置插入指定的新节点
void insert(List* pl,int pos,int data)
{
    //1.判断pos坐标是否合法
    if(pos < 0 || pos > pl->cnt)
    {
        //printf("插入坐标不合法，插入新节点失败\n");
        //return;
        //pos =0;默认插入到头节点位置
        pos = pl->cnt;//插入到尾部
    }
    //2.创建新节点
    //Node* pn = (Node*)malloc(sizeof(Node));
    //pn->data = data;
    //pn->next = NULL;
    Node* pn = create_node(data);
    //3.当向头节点位置插入节点时
    if(0 == pos)
    {
        pn->next = pl->head;
        pl->head = pn;
        pl->cnt++;
        return;
    }
    //4.当向其他位置插入新节点时
    Node* p = pl->head;
    int i = 0;
    for(i = 1; i < pos; i++)
    {
        p = p->next;//指向下一个
    }
    pn->next = p->next;
    p->next = pn;
    //节点个数 加1
    pl->cnt++;
}

//清空链表中的所有节点 clear
void clear(List* pl)
{
    while(pl->head != NULL)
    {
        //保存即将释放的节点地址
        Node* p = pl->head;
        //头指针指向下一个节点
        pl->head = pl->head->next;
        //释放保存的节点
        free(p);
        p = NULL;
    }
    //将链表中节点元素的个数置为0
    pl->cnt = 0;
}

//计算链表中节点的个数 size
int size(List* pl)
{
    return pl->cnt;
}

//判断链表是否为空 empty
int empty(List* pl)
{
    return NULL == pl->head;
}

//判断链表是否为满 full
int full(List* pl)
{
    return 0;
}

//遍历链表的操作
void travel(List* pl)
{
    Node* p = pl->head;
    printf("链表中的元素有：");
    while(p != NULL)
    {
        printf("%d ",p->data);
        //指向下一个
        p = p->next;
    }
    printf("\n");
}

//向链表的头节点位置插入新节点
void push_head(List* pl,int data)
{
    //1.创建新节点
    //Node* pn = (Node*)malloc(sizeof(Node));
    //pn->data = data;
    //pn->next = NULL;
    //Node* pn = create_node(data);
    //2.插入新节点到链表头部
    //pn->next = pl->head;
    //pl->head = pn;
    //3.节点个数加1
    //pl->cnt++;
    insert(pl,0,data);
}

