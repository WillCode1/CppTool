/* 动态分配内存练习，编写函数计算两个点的中间位置并把结果传递给调用函数
 * 要求使用动态分配内存记录中间点的位置
 */
#include<stdio.h>
#include<stdlib.h>

typedef struct {
    int row, col;
}pt;

pt *midpt(const pt *p_pt1, const pt *p_pt2){
    pt *p_mid = (pt *)malloc(sizeof(pt));
    if(p_mid){
        p_mid->row = (p_pt1->row + p_pt2->row) / 2;
        p_mid->col = (p_pt1->col + p_pt2->col) / 2;
    }
    return p_mid;
}

int main(){
    pt pt1 = {0}, pt2 = {0}, *p_mid = NULL;
    printf("请输入两个点的坐标：");
    scanf("%d%d%d%d", &(pt1.row), &(pt1.col), &(pt2.row),&(pt2.col));
    p_mid = midpt(&pt1, &pt2);
    if(p_mid){
        printf("中间点的坐标是：(%d, %d)\n", p_mid->row, p_mid->col);
        free(p_mid);
        p_mid = NULL;
    }
    return 0;
}
