/* 二级指针形参练习
 * 编写函数从两个圆里找出面积比较大的
 * 并把它传递给调用函数，这个函数不能使用返回值
 */
#include<stdio.h>

typedef struct {
    int row, col;
}pt;
typedef struct {
    int radius;
    pt center;
}circle;

void larger(const circle *p_c1, const circle *p_c2, circle **pp_c){
    *pp_c = (circle *)(p_c1->radius > p_c2->radius ? p_c1 : p_c2);
}

int main(){
    circle c1 = {0}, c2 = {0}, *p_c = NULL;
    printf("请输入第一个圆的坐标及半径：");
    scanf("%d%d%d", &(c1.center.row), &(c1.center.col), &(c1.radius));
    printf("请输入第二个圆的坐标及半径：");
    scanf("%d%d%d", &(c2.center.row), &(c2.center.col), &(c2.radius));
    larger(&c1, &c2, &p_c);
    printf("比较大的圆是：((%d, %d), %d)\n", p_c->center.row, p_c->center.col, p_c->radius);
    return 0;
}
