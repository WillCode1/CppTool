//编程实现有序二叉树的基本操作
#include <stdio.h>
#include <stdlib.h>

//定义节点的数据类型
typedef struct Node
{
    int data;//存储的具体内容
    struct Node* left;//左子树的地址
    struct Node* right;//右子树地址
}Node;

//定义二叉树的数据类型
typedef struct
{
    Node* root;//指向根节点的指针
    int cnt;//记录节点的个数
}Tree;

//插入新节点后，组成有序二叉树
void insertData(Tree* pt,int data);
//创建新节点
Node* create_node(int data);
//插入节点的递归函数
void insert(Node** pRoot,Node* pn);
//采用中序方法遍历二叉树
void travelData(Tree* pt);
//递归遍历函数
void travel(Node* pn);
//清空二叉树中所有的节点
void clearData(Tree* pt);
//清空的递归函数
void clear(Node** pRoot);
//查找指定元素所在的地址
Node** findData(Tree* pt,int data);
//查找的递归函数
Node** find(Node** pRoot,int data);
//删除指定的元素
void delData(Tree* pt,int data);
//修改二叉树中指定元素的值
void modifyData(Tree* pt,int data,int newData);
//判断二叉树是否为空
int empty(Tree* pt);
//判断二叉树是否为满
int full(Tree* pt);
//计算节点的个数
int size(Tree* pt);
//获取根节点元素值
int get_root(Tree* pt);
int main(void)
{
    //创建二叉树，并且进行初始化
    Tree tree;
    tree.root = NULL;
    tree.cnt = 0;
    
    insertData(&tree,50);
    travelData(&tree);//50
    insertData(&tree,70);
    travelData(&tree);//50 70
    insertData(&tree,20);
    travelData(&tree);//20 50 70
    insertData(&tree,60);
    travelData(&tree);//20 50 60 70
    
    printf("--------------------\n");
    delData(&tree,50);
    travelData(&tree);//20 60 70
    delData(&tree,50);
    travelData(&tree);//20 60 70

    printf("-------------------\n");
    modifyData(&tree,20,80);
    travelData(&tree);//60 70 80
    printf("二叉树中根节点元素是：%d\n",get_root(&tree));
    printf("二叉树中节点个数是：%d\n",size(&tree));
    printf("%s\n",empty(&tree)?"二叉树空了":"二叉树没有空");
    printf("%s\n",full(&tree)?"二叉树满了":"二叉树没有满");

    printf("--------------------\n");
    clearData(&tree);
    travelData(&tree);//啥也没有
    return 0;
}

//修改二叉树中指定元素的值
void modifyData(Tree* pt,int data,int newData)
{
    //1.删除指定元素的节点
    delData(pt,data);
    //2.插入新元素值
    insertData(pt,newData);
}

//判断二叉树是否为空
int empty(Tree* pt)
{
    return NULL == pt->root;
}

//判断二叉树是否为满
int full(Tree* pt)
{
    return 0;
}

//计算节点的个数
int size(Tree* pt)
{
    return pt->cnt;
}

//获取根节点元素值
int get_root(Tree* pt)
{
    /*
    if(empty(pt))
    {
        return -1;
    }
    return pt->root->data;
    */
    return empty(pt)?-1:pt->root->data;
}

//删除指定的元素
void delData(Tree* pt,int data)
{
    //1.查找目标元素所在的地址
    Node** p = findData(pt,data);
    //2.根据返回值进行判断,如果查找失败，则删除失败，函数结束
    if(NULL == *p)
    {
        printf("元素%d不存在，删除失败\n",data);
        return;
    }
    //3.如果查找成功,先将左子树合并到右子树中
    if((*p)->left != NULL)
    {
        insert(&(*p)->right,(*p)->left);
    }
    //4.将要删除的节点地址单独保存
    Node* q = *p;
    //5.将连接目标节点的指针指向合并出来的右子树
    *p = (*p)->right;
    //6.删除目标节点，个数减1
    free(q);
    q = NULL;
    pt->cnt--;
}

//查找的递归函数
Node** find(Node** pRoot,int data)
{
    //1.判断二叉树是否为空
    if(NULL == *pRoot)
    {
        return pRoot;//查找失败
    }
    //2.判断根节点是否和目标元素相等
    else if((*pRoot)->data == data)
    {
        return pRoot;//查找成功
    }
    //3.如果目标元素小于根节点，则去左子树中进行查找
    else if(data < (*pRoot)->data)
    {
        return find(&(*pRoot)->left,data);
    }
    //4.如果目标元素大于根节点，则去右子树中进行查找
    else 
    {
        return find(&(*pRoot)->right,data);
    }
}

//查找指定元素所在的地址
Node** findData(Tree* pt,int data)
{
    //调用递归函数实现查找
    return find(&pt->root,data);
}

//清空的递归函数
void clear(Node** pRoot)
{
    if(*pRoot != NULL)
    {
        //1.清空左子树
        clear(&(*pRoot)->left);
        //2.清空右子树
        clear(&(*pRoot)->right);
        //3.清空根节点
        free(*pRoot);
        *pRoot = NULL;
    }
}

//清空二叉树中所有的节点
void clearData(Tree* pt)
{
    //调用递归函数进行清空 址传递
    clear(&pt->root);
    //清空之后，节点个数为0
    pt->cnt = 0;
}

//递归遍历函数
void travel(Node* pn)
{
    if(pn != NULL)
    {
        //1.遍历左子树
        travel(pn->left);
        //2.遍历根节点
        printf("%d ",pn->data);
        //3.遍历右子树
        travel(pn->right);
    }
}

//采用中序方法遍历二叉树
void travelData(Tree* pt)
{
    //调用递归函数进行遍历
    travel(pt->root);
    printf("\n");
}

//插入节点的递归函数
void insert(Node** pRoot,Node* pn)
{
    //1.判断二叉树是否为空，如果为空，直接插入即可
    if(NULL == *pRoot)
    {
        //Node** pRoot = &root;
        //*pRoot = *(&root) = root;
        *pRoot = pn;
        return;
    }
    //2.如果二叉树不为空,则使用根节点与新节点比较大小
    //2.1 如果新节点小于根节点,则插入到左子树中
    if(pn->data < (*pRoot)->data)
    {
        insert(&(*pRoot)->left,pn);
    }
    //2.2 如果新节点大于等于根节点，则插入到右子树中
    else
    {
        insert(&(*pRoot)->right,pn);
    }

}

//创建新节点
Node* create_node(int data)
{
    Node* pn = (Node*)malloc(sizeof(Node));
    pn->data = data;
    pn->left = NULL;
    pn->right = NULL;
    return pn;
}

//插入新节点后，组成有序二叉树
void insertData(Tree* pt,int data)
{
    //1.创建新节点
    //2.插入新节点到二叉树中
    insert(&pt->root,create_node(data));
    //3.节点的个数加 1
    pt->cnt++;
}

