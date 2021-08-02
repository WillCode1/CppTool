/*
   二维数组演示
   多为数组中，首地址相同，则类型决定视野
   */
#include <stdio.h>
int main() {
    int arr[3][2] = {0};
    printf("arr可以表示二维数组首地址：%p\n", arr);
    printf("arr[0]表示二维数组中第一个数组地址：%p\n", arr[0]);
    printf("&arr[0][0]是数组中一个元素的地址：%p\n", &arr[0][0]);
    printf("sizeof(arr)是%d\n", sizeof(arr));
    printf("sizeof(arr[1])是%d\n", sizeof(arr[1]));
    return 0;
}
