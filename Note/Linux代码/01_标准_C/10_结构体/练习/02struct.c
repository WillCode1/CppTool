/* 结构体练习，
 * 编写函数从调用函数得到一个水平长方形的位置，
 * 计算长方形的面积并把结果传递给调用函数
 */
#include<stdio.h>

typedef struct {
    int row, col;
}pt;
typedef struct {
    pt pt1, pt2;
}rect;

int area(const rect *p_r1){
    int tmp = (p_r1->pt1.row - p_r1->pt2.row) * (p_r1->pt1.col - p_r1->pt2.col);
    return tmp > 0 ? tmp : 0 - tmp;
}

int main(){
    rect r1 = {0};
    printf("请输入水平长方形对角的两个坐标：");
    scanf("%d%d%d%d", &(r1.pt1.row), &(r1.pt1.col), &(r1.pt2.row), &(r1.pt2.col));
    printf("长方形面积是%d\n", area(&r1));
    return 0;
}
