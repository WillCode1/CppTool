/* 结构体练习，编写函数判断一个圆和一个点的位置关系
 * 判断结果有三种可能，点在圆外，点在圆上和点在圆内
 */
#include<stdio.h>

typedef struct {
    int row, col;
}pt;
typedef struct {
    int radius;
    pt center;
}circle;

void relation(const pt *p_pt1, const circle *p_circle){
    int tmp = 0;
    tmp = (p_pt1->row - p_circle->center.row) * (p_pt1->row - p_circle->center.row) + (p_pt1->col - p_circle->center.col) * (p_pt1->col - p_circle->center.col);
    if(tmp > p_circle->radius * p_circle->radius){
        printf("点在圆外\n");
    }
    else if(tmp == p_circle->radius * p_circle->radius){
        printf("点在圆上\n");
    }
    else{
        printf("点在圆内\n");
    }
}

int main(){
    pt pt1 = {0};
    circle c1 = {0};
    printf("请输入点的坐标：");
    scanf("%d%d", &(pt1.row), &(pt1.col));
    printf("请输入圆的坐标和半径：");
    scanf("%d%d%d", &c1.center.row, &c1.center.col, &c1.radius);
    relation(&pt1, &c1);
    return 0;
}
