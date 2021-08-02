/* 结构体练习，声明结构体用来记录屏幕上一个点的位置
 * 编写程序从键盘上得到一个水平长方形的位置，
 * 计算出它的中心点位置并把结果显示在屏幕上
 * 最后用指针改写
 */
#include<stdio.h>

typedef struct {
    int row, col;
}pt;
typedef struct {
    pt pt1, pt2;
}rect;

int main(){
    pt mid = {0}, *p_mid = NULL;
    rect r1 = {0}, *p_rect = NULL;
    p_mid = &mid;
    p_rect = &r1;
    printf("请输入水平长方形对角的两个坐标：");
//    scanf("%d%d%d%d", &(r1.pt1.row), &(r1.pt1.col), &(r1.pt2.row), &(r1.pt2.col));
//    mid.row = (r1.pt1.row + r1.pt2.row)/2;
//    mid.col = (r1.pt1.col + r1.pt2.col)/2;
//    printf("中心点坐标是(%d, %d)\n", mid.row, mid.col);
    scanf("%d%d%d%d", &(p_rect->pt1.row), &(p_rect->pt1.col), &(p_rect->pt2.row), &(p_rect->pt2.col));
    p_mid->row = (p_rect->pt1.row + p_rect->pt2.row)/2;
    p_mid->col = (p_rect->pt1.col + p_rect->pt2.col)/2;
    printf("中心点坐标是(%d, %d)\n", p_mid->row, p_mid->col);
    return 0;
}
