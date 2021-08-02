/*
   动态分配内存演示
   */
#include <stdio.h>
#include <stdlib.h>
typedef struct {
    int row, col;
} pt;
pt *read(void) {
    pt *p_pt = (pt *)malloc(sizeof(pt));
    if (p_pt) {
        printf("请输入点的位置：");
        scanf("%d%d", &(p_pt->row), &(p_pt->col));
    }
    return p_pt;
}
int main() {
    pt *p_pt = NULL;
    p_pt = read();
    if (p_pt) {
        printf("点的位置是(%d, %d)\n", p_pt->row, p_pt->col);
        free(p_pt);
        p_pt = NULL;
    }
    return 0;
}
