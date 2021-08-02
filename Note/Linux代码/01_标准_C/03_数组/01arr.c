/*
   数组演示
   计算机把数组当指针处理,相关地址的数据访问
   */
#include <stdio.h>
int main() {
    int arr[5] = {1, 2, 3, 4, 5};
    //数组名可以表示数组的第一个元素地址
    printf("arr可以表示a[0]地址：%p\n", arr);
    printf("&arr[0]表示a[0]地址：%p\n", &arr[0]);
    //计算机内部把数组当指针处理
    printf("&arr[4] = %p\n", arr + 4);
    printf("arr[4] = *(arr + 4) = %d\n", *(arr + 4));
    printf("sizeof(arr)是%d\n", sizeof(arr));
    return 0;
}
