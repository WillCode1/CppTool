/* typedef的使用方法
 * 新定义数据类型的别名就是脱去类型后的关键字，
 * 别名的数据类型就是除去typedef关键字后，定义的数据类型
 */
#include <stdio.h>

//1.定义有3个整型元素数组的类型的别名
typedef int array_t[3];

struct node{
    int data;
    struct node *next;
};
//2.定义结构体类型的别名
typedef struct node node_t;
//3.定义结构体指针类型的别名
typedef node_t *node_p;

//node_p p;   //结构体指针
//node_t **q; <==> node_p *q;//等价，二级结构体指针

//4.定义函数指针的别名
typedef int (*p_func)(int, int);

int main(){
    int i = 0;
    array_t arr[3] = {1,2,3};
    for(i = 0; i < 3; i++){
        printf("%d ", arr[i]);
    }
    printf("\n");
    return 0;
}
