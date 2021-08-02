/*
   结构体，结构体指针作函数形参演示
   输入结构体成员变量数据时，'&'后的数据加'()'
   */
#include <stdio.h>
typedef struct {
    int row, col;
} pt;
pt *read(pt *p_pt) {
    printf("请输入点的位置：");
    scanf("%d%d", &(p_pt->row), &(p_pt->col));
    return p_pt;
}
void print(const pt *p_pt) {
    printf("点的位置是(%d, %d)\n", p_pt->row, p_pt->col);
}
int main() {
    pt pt1 = {0}, *p_pt = NULL;
    p_pt = read(&pt1);
    print(p_pt);
    return 0;
}
