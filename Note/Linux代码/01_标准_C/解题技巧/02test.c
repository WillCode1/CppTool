/*输入数组，最大的与第一个交换，最小的与最后一个交换，输出数组
 */
#include<stdio.h>

int main(){
    int num = 0, tmp = 0, arr[8] = {0};
    int *p_max = arr, *p_mix = arr;
    printf("请输入8个整数：");
    for(num = 0; num < 8; num++){
        scanf("%d", arr + num);
        if(*p_max < *(arr + num)){
            p_max = arr + num;
        }
        if(*p_mix > *(arr + num)){
            p_mix = arr + num;
        }
    }
    if(p_max != arr){
        if(p_mix == arr) p_mix = p_max;
        tmp = *p_max;
        *p_max = *arr;
        *arr = tmp;
    }
    if(p_mix != arr + 7){
        if(p_max == arr + 7) p_max = p_mix;
        tmp = *p_mix;
        *p_mix = *(arr + 7);
        *(arr + 7) = tmp;
    }
    for(num = 0; num < 8; num++){
        printf("%d ", *(arr + num));
    }
    printf("\n");
    return 0;
}
