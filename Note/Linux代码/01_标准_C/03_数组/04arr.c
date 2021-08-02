//使用gdb调试,打印以下内容
#include<stdio.h>

int main(){
    int arr[] = {1,2,3,4,5};
    int brr[][3] = {1,2,3,4,5,6};
    //以下内容
    printf("%p\n", arr);
    printf("%p\n", arr+0);
    printf("%p\n", &arr);
    printf("%p\n", &arr[1]);
    printf("%p\n", &brr);
    printf("%p\n", &brr[1]);
    printf("%p\n", &brr[1][1]);
    return 0;
}
