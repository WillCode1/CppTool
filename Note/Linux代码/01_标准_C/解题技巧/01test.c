/*输入三个数，按从小到大的顺序排列并输出,使用指针
 */
#include<stdio.h>

int main(){
    int a = 0, b = 0, c = 0;
    int *max = &a, *mid = &b, *mix = &c, *p_tmp = NULL;
    printf("请输入三个整数：");
    scanf("%d%d%d", &a, &b, &c);
    if(*max < *mid){
        p_tmp = max;
        max = mid;
        mid = p_tmp;
    }
    if(*max < *mix){
        p_tmp = max;
        max = mix;
        mix = p_tmp;
    }
    if(*mix > *mid){
        p_tmp = mix;
        mix = mid;
        mid = p_tmp;
    }
    printf("%d %d %d\n",*mix, *mid, *max);
    return 0;
}
