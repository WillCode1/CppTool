/* 指针练习,高级应用，改变指针指向
 * 输入3个数字，从小到大打印，尽量使用指针完成，并且不改变变量内容
 */
#include<stdio.h>

int main(){
    int num = 0, num1 = 0, num2 = 0;
    printf("请输入3个数字：");
    scanf("%d%d%d", &num, &num1, &num2);
    int *p_max = &num, *p_mid = &num1, *p_mix = &num2;
    int *p_tmp = NULL;
    if(*p_max < *p_mid){
        p_tmp = p_max;
        p_max = p_mid;
        p_mid = p_tmp;
    }
    if(*p_max < *p_mix){
        p_tmp = p_max;
        p_max = p_mix;
        p_mix = p_tmp;
    }
    if(*p_mid < *p_mix){
        p_tmp = p_mid;
        p_mid = p_mix;
        p_mix = p_tmp;
    }
    printf("输入数字从小到大依次是：%d %d %d\n", *p_mix, *p_mid, *p_max);
    return 0;
}
