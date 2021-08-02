/*有8个整数，输入n，前面整数向后移动n位，后面n个数字移到最前
 */
#include<stdio.h>

int main(){
    int num = 0, m = 0, tmp = 0;
    int arr[] = {1,2,3,4,5,6,7,8}, arr1[8] = {0};
    printf("请输入一个1~8的数字：");
    scanf("%d", &m);
    for(num = 0; num < 8 - m; num++){
        arr1[num + m] = arr[num];
    }
    for(num = 0; num < m; num++){
        arr1[num] = arr[num + 8 - m];
    }
    for(num = 0; num < 8; num++){
        printf("%d ", arr1[num]);
    }
    printf("\n");
    return 0;
}
