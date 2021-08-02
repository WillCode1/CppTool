/* 结构体练习，编写函数从两个圆里找到面积比较大的那个
 * 并把结果传递给调用函数
 * 注意使用const
 */
#include<stdio.h>

typedef struct {
    int row, col;
}pt;
typedef struct {
    int radius;
    pt center;
}circle;

circle *compare(const circle *c1, const circle *c2){
    return c1->radius > c2->radius ? c1 : c2;
}

int main(){
    circle c1 = {0}, c2 = {0};
    circle *p_c = NULL;
    printf("请输入第一个圆的坐标和半径：");
    scanf("%d%d%d", &(c1.center.row), &(c1.center.col), &(c1.radius));
    printf("请输入第二个圆的坐标和半径：");
    scanf("%d%d%d", &(c2.center.row), &(c2.center.col), &(c2.radius));
    p_c = compare(&c1, &c2);
    printf("面积比较大的圆坐标和半径是：(%d, %d)和%d\n", p_c->center.row, p_c->center.col, p_c->radius);
    return 0;
}
